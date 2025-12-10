#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class ForwardPositionPublisher(Node):
    def __init__(self):
        super().__init__('forward_position_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.5, 0.0, 0.0, 0.0, 0.0],
            [0.5, 0.3, -0.2, 0.0, 0.5, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        self.current_index = 0
        self.timer = self.create_timer(2.0, self.publish_positions)
        self.get_logger().info('ForwardPositionPublisher pokrenut, šalje test pozicije!')

    def publish_positions(self):
        pos = self.positions[self.current_index]
        msg = Float64MultiArray()
        msg.data = pos
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent: {pos}')
        self.current_index += 1
        if self.current_index >= len(self.positions):
            self.get_logger().info('Sve pozicije poslane. Node završava.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ForwardPositionPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
