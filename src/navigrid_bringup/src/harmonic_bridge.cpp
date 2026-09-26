#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/clock.pb.h>

class HarmonicBridge : public rclcpp::Node
{
public:
  HarmonicBridge()
  : Node("harmonic_bridge")
  {
    // ROS Publishers
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ROS Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        gz::msgs::Twist gz_twist;
        gz_twist.mutable_linear()->set_x(msg->linear.x);
        gz_twist.mutable_linear()->set_y(msg->linear.y);
        gz_twist.mutable_linear()->set_z(msg->linear.z);
        gz_twist.mutable_angular()->set_x(msg->angular.x);
        gz_twist.mutable_angular()->set_y(msg->angular.y);
        gz_twist.mutable_angular()->set_z(msg->angular.z);
        cmd_vel_pub_gz_.Publish(gz_twist);
      });

    secondary_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/secondary_amr/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        gz::msgs::Twist gz_twist;
        gz_twist.mutable_linear()->set_x(msg->linear.x);
        gz_twist.mutable_linear()->set_y(msg->linear.y);
        gz_twist.mutable_linear()->set_z(msg->linear.z);
        gz_twist.mutable_angular()->set_x(msg->angular.x);
        gz_twist.mutable_angular()->set_y(msg->angular.y);
        gz_twist.mutable_angular()->set_z(msg->angular.z);
        sec_cmd_pub_gz_.Publish(gz_twist);
      });

    // Gazebo Transport Publishers
    cmd_vel_pub_gz_ = gz_node_.Advertise<gz::msgs::Twist>("/cmd_vel");
    sec_cmd_pub_gz_ = gz_node_.Advertise<gz::msgs::Twist>("/secondary_amr/cmd_vel");

    // Gazebo Transport Subscribers
    gz_node_.Subscribe("/scan", &HarmonicBridge::onGzScan, this);
    gz_node_.Subscribe("/imu", &HarmonicBridge::onGzImu, this);
    gz_node_.Subscribe("/odom", &HarmonicBridge::onGzOdom, this);
    gz_node_.Subscribe("/clock", &HarmonicBridge::onGzClock, this);

    RCLCPP_INFO(this->get_logger(), "Native Gazebo Harmonic (GZ Sim 8) Bridge initialized successfully.");
  }

private:
  void onGzScan(const gz::msgs::LaserScan & gz_msg)
  {
    sensor_msgs::msg::LaserScan ros_msg;
    ros_msg.header.stamp = this->now();
    ros_msg.header.frame_id = "lidar_link";

    ros_msg.angle_min = static_cast<float>(gz_msg.angle_min());
    ros_msg.angle_max = static_cast<float>(gz_msg.angle_max());
    ros_msg.angle_increment = static_cast<float>(gz_msg.angle_step());
    ros_msg.time_increment = 0.0f;
    ros_msg.scan_time = 0.05f;
    ros_msg.range_min = static_cast<float>(gz_msg.range_min());
    ros_msg.range_max = static_cast<float>(gz_msg.range_max());

    ros_msg.ranges.reserve(gz_msg.ranges_size());
    for (int i = 0; i < gz_msg.ranges_size(); ++i) {
      ros_msg.ranges.push_back(static_cast<float>(gz_msg.ranges(i)));
    }

    ros_msg.intensities.reserve(gz_msg.intensities_size());
    for (int i = 0; i < gz_msg.intensities_size(); ++i) {
      ros_msg.intensities.push_back(static_cast<float>(gz_msg.intensities(i)));
    }

    scan_pub_->publish(ros_msg);
  }

  void onGzImu(const gz::msgs::IMU & gz_msg)
  {
    sensor_msgs::msg::Imu ros_msg;
    ros_msg.header.stamp = this->now();
    ros_msg.header.frame_id = "imu_link";

    ros_msg.orientation.x = gz_msg.orientation().x();
    ros_msg.orientation.y = gz_msg.orientation().y();
    ros_msg.orientation.z = gz_msg.orientation().z();
    ros_msg.orientation.w = gz_msg.orientation().w();

    ros_msg.angular_velocity.x = gz_msg.angular_velocity().x();
    ros_msg.angular_velocity.y = gz_msg.angular_velocity().y();
    ros_msg.angular_velocity.z = gz_msg.angular_velocity().z();

    ros_msg.linear_acceleration.x = gz_msg.linear_acceleration().x();
    ros_msg.linear_acceleration.y = gz_msg.linear_acceleration().y();
    ros_msg.linear_acceleration.z = gz_msg.linear_acceleration().z();

    imu_pub_->publish(ros_msg);
  }

  void onGzOdom(const gz::msgs::Odometry & gz_msg)
  {
    rclcpp::Time current_time = this->now();

    nav_msgs::msg::Odometry ros_msg;
    ros_msg.header.stamp = current_time;
    ros_msg.header.frame_id = "odom";
    ros_msg.child_frame_id = "base_footprint";

    ros_msg.pose.pose.position.x = gz_msg.pose().position().x();
    ros_msg.pose.pose.position.y = gz_msg.pose().position().y();
    ros_msg.pose.pose.position.z = gz_msg.pose().position().z();
    ros_msg.pose.pose.orientation.x = gz_msg.pose().orientation().x();
    ros_msg.pose.pose.orientation.y = gz_msg.pose().orientation().y();
    ros_msg.pose.pose.orientation.z = gz_msg.pose().orientation().z();
    ros_msg.pose.pose.orientation.w = gz_msg.pose().orientation().w();

    ros_msg.twist.twist.linear.x = gz_msg.twist().linear().x();
    ros_msg.twist.twist.linear.y = gz_msg.twist().linear().y();
    ros_msg.twist.twist.linear.z = gz_msg.twist().linear().z();
    ros_msg.twist.twist.angular.x = gz_msg.twist().angular().x();
    ros_msg.twist.twist.angular.y = gz_msg.twist().angular().y();
    ros_msg.twist.twist.angular.z = gz_msg.twist().angular().z();

    odom_pub_->publish(ros_msg);

    // Broadcast odom -> base_footprint TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = gz_msg.pose().position().x();
    tf_msg.transform.translation.y = gz_msg.pose().position().y();
    tf_msg.transform.translation.z = gz_msg.pose().position().z();
    tf_msg.transform.rotation = ros_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void onGzClock(const gz::msgs::Clock & gz_msg)
  {
    rosgraph_msgs::msg::Clock ros_msg;
    ros_msg.clock.sec = static_cast<int32_t>(gz_msg.sim().sec());
    ros_msg.clock.nanosec = static_cast<uint32_t>(gz_msg.sim().nsec());
    clock_pub_->publish(ros_msg);
  }

  // ROS handles
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr secondary_cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // GZ handles
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher cmd_vel_pub_gz_;
  gz::transport::Node::Publisher sec_cmd_pub_gz_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HarmonicBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
