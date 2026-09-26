#!/usr/bin/env python3
"""
Secondary AMR Patrol Node
Drives the secondary autonomous mobile robot back and forth along a fixed trajectory
perpendicular to the main route, serving as a dynamic moving obstacle.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AMRPatrolNode(Node):
    def __init__(self):
        super().__init__('amr_patrol_node')
        self.cmd_pub = self.create_publisher(Twist, '/secondary_amr/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.speed = 0.5  # m/s
        self.period = 6.0  # seconds per half-cycle
        self.elapsed = 0.0
        self.direction = 1.0

        self.get_logger().info('AMR Patrol Node initialized. Patrolling dynamically.')

    def timer_callback(self):
        dt = 0.05
        self.elapsed += dt

        if self.elapsed >= self.period:
            self.elapsed = 0.0
            self.direction *= -1.0

        msg = Twist()
        msg.linear.x = self.speed * self.direction
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AMRPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
