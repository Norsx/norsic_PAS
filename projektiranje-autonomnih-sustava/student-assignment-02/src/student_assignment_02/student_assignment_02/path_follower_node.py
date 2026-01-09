#!/usr/bin/env python3
"""
Path Follower Node
Koristi Pure Pursuit algoritam za slijeđenje putanje generirane A* planaterom
Objavljuje cmd_vel zapovijedima za robot
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from typing import Tuple, Optional


class PathFollowerNode(Node):
    """ROS2 čvor za slijeđenje putanje koristeći Pure Pursuit algoritam"""
    
    def __init__(self):
        super().__init__('path_follower')
        
        # QoS profil za path
        qos_path = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe na putanju od A* planera
        self.path_subscription = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            qos_path
        )
        
        # Publisher za cmd_vel (zapovijedne brzine)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Timer za procesiranje putanje
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Parametri
        self.declare_parameter('lookahead_distance', 0.3)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_tolerance', 0.1)  # Tolerancija do cilja (m)
        
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # Trenutna putanja
        self.current_path: Optional[Path] = None
        self.path_index = 0
        self.robot_position = (0.0, 0.0)
        
        self.get_logger().info(
            f'Path Follower Node inicijaliziran:\n'
            f'  - Lookahead distanca: {self.lookahead_distance}m\n'
            f'  - Max linearna brzina: {self.max_linear_velocity}m/s\n'
            f'  - Max kutna brzina: {self.max_angular_velocity}rad/s\n'
            f'  - Goal tolerancija: {self.goal_tolerance}m'
        )
    
    def path_callback(self, msg: Path):
        """Prima novu putanju od A* planera"""
        self.current_path = msg
        self.path_index = 0
        self.get_logger().info(f'Nova putanja primljena! Dužina: {len(msg.poses)} točaka')
    
    def get_robot_position(self) -> Tuple[float, float]:
        """
        Proba dobiti poziciju robota iz TF-a
        Ako ne može, koristi zadnju poznatu poziciju
        """
        try:
            from tf2_ros import TransformListener, Buffer
            from tf2_geometry_msgs import do_transform_point
            
            # Ovdje bi trebao dohvatiti base_link poziciju iz TF-a
            # Za sada koristi prva točka putanje kao aproximaciju
            if self.current_path and len(self.current_path.poses) > 0:
                pose = self.current_path.poses[0]
                return (pose.pose.position.x, pose.pose.position.y)
            
            return self.robot_position
        except Exception as e:
            self.get_logger().warn(f'Ne mogu dobiti poziciju robota: {e}')
            return self.robot_position
    
    def find_lookahead_point(self) -> Optional[Tuple[float, float]]:
        """
        Pronađi točku na putanji koja je na udaljenosti lookahead_distance
        od robota (Pure Pursuit algoritam)
        """
        if not self.current_path or len(self.current_path.poses) == 0:
            return None
        
        # Dobij trenutnu poziciju robota
        robot_x, robot_y = self.get_robot_position()
        
        # Pronađi točku na putanji najbližu robotu
        min_distance = float('inf')
        closest_index = 0
        
        for i, pose in enumerate(self.current_path.poses):
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # Pronađi točku na udaljenosti lookahead_distance od robota
        cumulative_distance = 0.0
        for i in range(closest_index, len(self.current_path.poses) - 1):
            pose_a = self.current_path.poses[i]
            pose_b = self.current_path.poses[i + 1]
            
            dx = pose_b.pose.position.x - pose_a.pose.position.x
            dy = pose_b.pose.position.y - pose_a.pose.position.y
            segment_distance = math.sqrt(dx*dx + dy*dy)
            
            cumulative_distance += segment_distance
            
            if cumulative_distance >= self.lookahead_distance:
                # Interpola točku na segmentu
                # Za sada koristi samo pose_b
                return (pose_b.pose.position.x, pose_b.pose.position.y)
        
        # Ako nema točke na lookahead distanci, koristi zadnju
        last_pose = self.current_path.poses[-1]
        return (last_pose.pose.position.x, last_pose.pose.position.y)
    
    def calculate_control_command(self, lookahead_point: Tuple[float, float]) -> Twist:
        """
        Izračunaj zapovijedne brzine koristeći Pure Pursuit
        
        Pure Pursuit:
        1. Pronađi lookahead točku na putanji
        2. Izračunaj kutnu grješku
        3. Konvertuj u kutnu brzinu
        """
        cmd = Twist()
        
        if not lookahead_point:
            return cmd
        
        robot_x, robot_y = self.get_robot_position()
        
        # Vektor do lookahead točke
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        # Ako je robot na cilju, stop
        if distance_to_target < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd
        
        # Linearna brzina - ide prema cilju
        # Smanji brzinu ako je blizu cilja
        if distance_to_target < 0.5:
            cmd.linear.x = distance_to_target / 0.5 * self.max_linear_velocity
        else:
            cmd.linear.x = self.max_linear_velocity
        
        # Kutna brzina - Pure Pursuit
        # Kutni error: -pi do pi
        target_heading = math.atan2(dy, dx)
        current_heading = 0.0  # Trebalo bi dobiti iz robota
        
        angular_error = target_heading - current_heading
        
        # Normaliziraj kutni error na -pi do pi
        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi
        
        # Proporcionalna kontrola
        kp = 1.0  # Proporcionalni gain
        cmd.angular.z = kp * angular_error
        
        # Ograniči kutnu brzinu
        cmd.angular.z = max(-self.max_angular_velocity, 
                           min(self.max_angular_velocity, cmd.angular.z))
        
        return cmd
    
    def timer_callback(self):
        """
Timer callback - redovito procesira putanju i objavljuje cmd_vel
        """
        if not self.current_path or len(self.current_path.poses) == 0:
            # Nema putanje, stop
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
            return
        
        # Pronađi lookahead točku
        lookahead_point = self.find_lookahead_point()
        
        if not lookahead_point:
            self.get_logger().warn('Ne mogu pronaći lookahead točku')
            return
        
        # Izračunaj zapovijedne brzine
        cmd = self.calculate_control_command(lookahead_point)
        
        # Objavi zapovijedne brzine
        self.cmd_vel_publisher.publish(cmd)
        
        # Debug info
        self.get_logger().debug(
            f'Linear: {cmd.linear.x:.2f} m/s, '
            f'Angular: {cmd.angular.z:.2f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
