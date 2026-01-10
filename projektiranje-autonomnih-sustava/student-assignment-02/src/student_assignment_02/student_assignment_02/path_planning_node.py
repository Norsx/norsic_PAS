#!/usr/bin/env python3
"""
Path Planning Node - Dummy čvor
Provjena: entry_points test
"""

import rclpy
from rclpy.node import Node

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        self.get_logger().info('Path Planning Node: Started (dummy node for testing)')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
