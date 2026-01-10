#!/usr/bin/env python3
"""
Goal Navigation Node - Dummy čvor
Provjeravanje: entry_points test
"""

import rclpy
from rclpy.node import Node

class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation_node')
        self.get_logger().info('Goal Navigation Node: Started (dummy node for testing)')

def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
