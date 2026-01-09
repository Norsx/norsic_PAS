#!/usr/bin/env python3
"""
Nav2 Adapter Node - JEDNOSTAVNIJA VERZIJA
BEZ /follow_path akcije (koja zahtijeva controller_server)

FIX: Direktno slijedi putanju bez Nav2 kontrolera
- Hvata /planned_path od A* planera
- Šalje /cmd_vel komande za kretanje
- Robot se kreće prema cilju

Ovakvo je jednostavnije i ne zahtijeva:
- Lifecycle Manager
- Controller Server
- DWB Controller
- Local Costmap

Samo:
1. Hvata putanju
2. Prati je
3. Šalje komande robotu
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import TransformListener, Buffer
import math
import time


class Nav2Adapter(Node):
    """Adapter koji sljedi putanju bez controller_server"""
    
    def __init__(self):
        super().__init__('nav2_adapter')
        
        self.get_logger().info(
            '\n' +
            '='*80 +
            '\n[ADAPTER] INICIJALIZACIJA - JEDNOSTAVNA VERZIJA' +
            '\n' +
            '- Sluša: /planned_path (od A* planera)' +
            '\n' +
            '- Šalje: /cmd_vel (direktno robotu)' +
            '\n' +
            '- NEMA /follow_path akcije' +
            '\n' +
            '- NEMA controller_server ovisnosti' +
            '\n' +
            '='*80 +
            '\n'
        )
        
        # Subscribe na A* putanju
        self.path_subscription = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        self.get_logger().info('[ADAPTER] ✓ Subscribe na /planned_path')
        
        # Publisher za čv komande
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.get_logger().info('[ADAPTER] ✓ Publisher za /cmd_vel')
        
        # TF buffer za transformacije
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Skladistenje putanje
        self.current_path: Path = None
        self.path_index = 0
        self.following = False
        
        # Parametri kretanja
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.position_tolerance = 0.1  # m
        self.angle_tolerance = 0.2  # rad
        
        # Timer za sljedićenje putanje
        self.timer = self.create_timer(0.1, self.follow_path_timer)
        
        self.get_logger().info('[ADAPTER] ✓ Inicijalizacija gotova')
        self.get_logger().info('')
    
    def path_callback(self, msg: Path):
        """Prima putanju od A* planera"""
        if len(msg.poses) > 0:
            self.current_path = msg
            self.path_index = 0
            self.following = True
            length = self.calculate_path_length(msg)
            
            self.get_logger().info(
                f'[PATH] ✓ Primljena putanja: {len(msg.poses)} točaka, '
                f'dužina: {length:.2f}m'
            )
            self.get_logger().info(
                f'[FOLLOW] ✓ Počinjem sljedićenje putanje...'
            )
        else:
            self.get_logger().warn('[PATH] ✗ Putanja je PRAZNA!')
            self.following = False
    
    def calculate_path_length(self, path: Path) -> float:
        """Izračuna dužinu putanje"""
        if len(path.poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total_length += math.sqrt(dx*dx + dy*dy)
        
        return total_length
    
    def follow_path_timer(self):
        """Timer za sljedićenje putanje"""
        if not self.following or self.current_path is None:
            # Zaustavi robota
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
            return
        
        # Provjeri je li dostignut kraj putanje
        if self.path_index >= len(self.current_path.poses):
            self.get_logger().info('[DONE] ✓ SLJEDIĆENJE DOVRŠENO')
            self.following = False
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
            return
        
        # Dohvati robot poziciju
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception:
            # Ako nema transformacije, zaustavi robota
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
            return
        
        # Dohvati trenutnu čeljnu točku na putanji
        target_pose = self.current_path.poses[self.path_index]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # Izračuna udaljenost do čelje
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Ako je suvisze blizu, idi na sljedeću točku
        if distance < self.position_tolerance:
            self.path_index += 1
            return
        
        # Izračuna ugao prema čelji
        angle_to_target = math.atan2(dy, dx)
        
        # Izračuna razliku kuta
        # (ovo je pojednostavljeno - trebalo bi koristiti robot orientaciju)
        
        # Kreiraj Twist komandu
        cmd = Twist()
        
        # Linearna brzina proporcionalna udaljenosti
        cmd.linear.x = min(self.max_linear_speed, distance * 0.5)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        
        # Kutna brzina (pojednostavljeno)
        cmd.angular.z = angle_to_target * 0.5
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        
        # Objavi komandu
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zaustavi robota prije gasišenja
        cmd = Twist()
        node.cmd_vel_publisher.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
