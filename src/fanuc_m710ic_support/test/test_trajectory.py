#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryTestNode(Node):
    def __init__(self):
        super().__init__('trajectory_test_node')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        goal_msg.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)
        
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.3, -0.2, 0.0, 0.5, 0.0]
        point2.time_from_start = Duration(sec=2, nanosec=0)
        
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
        point3.time_from_start = Duration(sec=4, nanosec=0)
        
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point4.time_from_start = Duration(sec=6, nanosec=0)
        
        goal_msg.trajectory.points = [point1, point2, point3, point4]
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        self.get_logger().info('Sending trajectory goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTestNode()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
