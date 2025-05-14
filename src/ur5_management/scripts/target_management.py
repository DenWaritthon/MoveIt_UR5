#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ur5_interfaces.srv import *

class TargetManagement(Node):
    def __init__(self):
        super().__init__('target_management')

        # Create a service server
        self.create_service(Start, '/start', self.start_callback)
        self.create_service(MoveItReport, '/ur5_report', self.report_callback)

        # Create a service client
        self.get_target_group = MutuallyExclusiveCallbackGroup()
        self.get_target_client = self.create_client(GetTarget, '/get_target',callback_group = self.get_target_group)

        self.moveit_target_group = MutuallyExclusiveCallbackGroup()
        self.moveit_target_client = self.create_client(MoveItTarget, '/target',callback_group = self.moveit_target_group)

        while not self.get_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /get_target not available, waiting again...')
        self.get_logger().info('service /get_target is available')

        self.get_logger().info(f'Target Management Node has been started')

        # Variables
        self.pick_target = [0.0, 0.0, 0.0]
        self.place_target = [0.0, 0.0, 0.0]
        

    def start_callback(self, request:Start.Request, response:Start.Response):
        self.get_logger().info(f'Start call')
        # Call get target
        self.call_get_target()
        self.call_moveit_target()
        return response
    
    def report_callback(self, request:MoveItReport.Request, response:MoveItReport.Response):
        self.get_logger().info(f'Report call')

        self.get_logger().info(f'Move done: {request.move_done}')
        self.get_logger().info(f'Curent pose: {request.current_pose}')
        return response
    
    def call_moveit_target(self):
        request = MoveItTarget.Request()
        request.target.position.x = self.pick_target[0]
        request.target.position.y = self.pick_target[1]
        request.target.position.z = self.pick_target[2]

        result = self.moveit_target_client.call(request)

        self.get_logger().info(f'Set target done: {result.success}')

    def call_get_target(self):
        request = GetTarget.Request()
        request.call = True
        result = self.get_target_client.call(request)

        self.pick_target[0] = result.pick_target.position.x
        self.pick_target[1] = result.pick_target.position.y
        self.pick_target[2] = result.pick_target.position.z

        self.place_target[0] = result.place_target.position.x
        self.place_target[1] = result.place_target.position.y
        self.place_target[2] = result.place_target.position.z

        self.get_logger().info(f'Pick target: {self.pick_target}')
        self.get_logger().info(f'Place target: {self.place_target}')
        self.get_logger().info(f'Get target success')

def main(args=None):
    rclpy.init(args=args)
    node = TargetManagement()
    # rclpy.spin(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
