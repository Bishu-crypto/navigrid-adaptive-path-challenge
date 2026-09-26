#!/usr/bin/env python3
"""
NaviGrid Safety Override Node
Monitors 2D LIDAR (/scan) and robot odometry (/odom) to enforce a dynamic stopping distance:
    d_safe = k * v^2 + d_min
If an obstacle enters the dynamic stopping envelope, this node publishes zero velocity to
/cmd_vel_safety at highest priority to override autonomous navigation commands.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SafetyOverrideNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Declare ROS 2 parameters
        self.declare_parameter('k', 0.5)
        self.declare_parameter('d_min', 0.5)
        self.declare_parameter('cone_angle_deg', 120.0)  # Front detection cone
        self.declare_parameter('override_rate_hz', 20.0)

        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.d_min = self.get_parameter('d_min').get_parameter_value().double_value
        self.cone_angle = math.radians(self.get_parameter('cone_angle_deg').get_parameter_value().double_value)

        # State variables
        self.current_speed = 0.0
        self.last_scan = None
        self.override_active = False

        # Publishers & Subscribers
        self.safety_cmd_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        timer_period = 1.0 / self.get_parameter('override_rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'NaviGrid Safety Node started. Parameters: k={self.k}, d_min={self.d_min}m, cone={math.degrees(self.cone_angle):.1f}°'
        )

    def odom_callback(self, msg: Odometry):
        # Calculate linear speed v in robot frame
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def control_loop(self):
        # Dynamically fetch parameters in case changed at runtime
        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.d_min = self.get_parameter('d_min').get_parameter_value().double_value

        # Dynamic safety distance equation: d_safe = k * v^2 + d_min
        d_safe = self.k * (self.current_speed ** 2) + self.d_min

        if self.last_scan is None:
            return

        scan = self.last_scan
        half_cone = self.cone_angle / 2.0
        min_obstacle_dist = float('inf')

        # Check all beams inside front cone
        angle = scan.angle_min
        for r in scan.ranges:
            if not math.isnan(r) and not math.isinf(r) and r >= scan.range_min and r <= scan.range_max:
                # Check if angle is within front detection cone [-half_cone, +half_cone]
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                if abs(norm_angle) <= half_cone:
                    if r < min_obstacle_dist:
                        min_obstacle_dist = r
            angle += scan.angle_increment

        # Safety override evaluation
        if min_obstacle_dist < d_safe:
            # Publish emergency zero-velocity override
            zero_twist = Twist()
            self.safety_cmd_pub.publish(zero_twist)

            if not self.override_active:
                self.override_active = True
                self.get_logger().warn(
                    f'EMERGENCY SAFETY OVERRIDE ACTIVE! '
                    f'Obstacle distance: {min_obstacle_dist:.3f}m < d_safe: {d_safe:.3f}m '
                    f'(speed: {self.current_speed:.2f}m/s, k={self.k}, d_min={self.d_min}m)'
                )
            else:
                self.get_logger().info(
                    f'[OVERRIDE] Obstacle: {min_obstacle_dist:.2f}m | d_safe: {d_safe:.2f}m | v: {self.current_speed:.2f}m/s',
                    throttle_duration_sec=1.0
                )
        else:
            if self.override_active:
                self.override_active = False
                self.get_logger().info(
                    f'SAFETY OVERRIDE CLEARED. '
                    f'Obstacle distance: {min_obstacle_dist:.3f}m >= d_safe: {d_safe:.3f}m. Resuming navigation.'
                )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
