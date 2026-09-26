#!/usr/bin/env python3
"""
Twist Priority Multiplexer Node
Arbitrates between high-priority safety override (/cmd_vel_safety) and
standard autonomous navigation commands (/cmd_vel_nav), forwarding the
highest-priority active command to the robot drive topic (/cmd_vel).
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist


class TwistPriorityMux(Node):
    def __init__(self):
        super().__init__('twist_priority_mux')

        self.declare_parameter('safety_timeout_sec', 0.3)
        self.declare_parameter('nav_timeout_sec', 0.5)

        self.safety_timeout = self.get_parameter('safety_timeout_sec').get_parameter_value().double_value
        self.nav_timeout = self.get_parameter('nav_timeout_sec').get_parameter_value().double_value

        self.last_safety_time = None
        self.last_safety_msg = None

        self.last_nav_time = None
        self.last_nav_msg = None

        self.output_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Twist, '/cmd_vel_safety', self.safety_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cb, 10)

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('Twist Priority Mux started: /cmd_vel_safety overrides /cmd_vel_nav -> /cmd_vel')

    def safety_cb(self, msg: Twist):
        self.last_safety_time = self.get_clock().now()
        self.last_safety_msg = msg

    def nav_cb(self, msg: Twist):
        self.last_nav_time = self.get_clock().now()
        self.last_nav_msg = msg

    def timer_cb(self):
        now = self.get_clock().now()
        safety_active = False

        if self.last_safety_time is not None:
            dt_safety = (now - self.last_safety_time).nanoseconds / 1e9
            if dt_safety <= self.safety_timeout:
                safety_active = True

        if safety_active and self.last_safety_msg is not None:
            # High priority safety active
            self.output_pub.publish(self.last_safety_msg)
            return

        # Check navigation
        if self.last_nav_time is not None:
            dt_nav = (now - self.last_nav_time).nanoseconds / 1e9
            if dt_nav <= self.nav_timeout and self.last_nav_msg is not None:
                self.output_pub.publish(self.last_nav_msg)
                return

        # No active sources: publish zero or idle
        # We don't flood when idle unless required


def main(args=None):
    rclpy.init(args=args)
    node = TwistPriorityMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
