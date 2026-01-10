#!/usr/bin/env python3
"""
Map Republisher Node
Re-objavljuje /map kontinuirano s Transient Local QoS

Rješava problem gdje nav2_map_server ne objavljuje /map kontinuirano
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # QoS za /map (Transient Local)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe na /map s Transient Local
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # Republish na /map s kontinuiranom frekvencijom
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )
        
        self.latest_map = None
        self.get_logger().info('Map Republisher: Started')
        
        # Republish svaku sekundu
        self.timer = self.create_timer(1.0, self.republish_map)
    
    def map_callback(self, msg):
        self.latest_map = msg
        self.get_logger().info(f'Map Republisher: Received map ({msg.info.width}x{msg.info.height})')
    
    def republish_map(self):
        if self.latest_map is not None:
            self.map_publisher.publish(self.latest_map)

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
