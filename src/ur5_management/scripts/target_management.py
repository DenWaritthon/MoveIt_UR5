#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python import get_package_share_directory

from ur5_interfaces.srv import *
import yaml
import os

class TargetManagement(Node):
    def __init__(self):
        super().__init__('target_management')

        # Set up .yaml file
        pkg_name = 'ur5_management'
        pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = pkg_share_path.split('install')
        self.yaml_path = os.path.join(ws_path ,'src', pkg_name , 'data_logger', 'move_log.yaml')

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

        while not self.moveit_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /target not available, waiting again...')
        self.get_logger().info('service /target is available')

        # Create a timer
        self.timer = self.create_timer(0.1, self.timer_loop)

        self.get_logger().info(f'Target Management Node has been started')

        # Variables
        self.start = False
        self.move_done = False
        self.curent_pose = [0.0, 0.0, 0.0]
        self.move_count_max = 18
        self.move_count = 0
        self.move_step = 0
        
        self.move_pick = 0.0
        self.move_place = 0.0

        self.home_target = [0.45, -0.1, 0.5]
        self.pick_target = [0.0, 0.0, 0.0]
        self.place_target = [0.0, 0.0, 0.0]

        self.target_list = []

    def start_callback(self, request:Start.Request, response:Start.Response):
        # self.get_logger().info(f'Start call')
        if request.start:
            self.start = True
            self.move_done = True
            self.get_logger().info(f'Start: {self.start}')
            self.get_logger().info(f'Start time : {self.get_clock().now().to_msg()}')
            self.call_get_target()
            self.set_target_list()

        return response
    
    def report_callback(self, request:MoveItReport.Request, response:MoveItReport.Response):
        # self.get_logger().info(f'Report call')
        if request.move_done:
            self.move_done = True
            self.curent_pose[0] = request.current_pose.position.x
            self.curent_pose[1] = request.current_pose.position.y
            self.curent_pose[2] = request.current_pose.position.z
            # self.get_logger().info(f'Move done: {self.move_done}')

            if self.move_step == 3:
                self.get_logger().info(f'Pick target reached')
                self.save_yaml(pick=True)
            elif self.move_step == 7:
                self.get_logger().info(f'Place target reached')
                self.save_yaml(place=True)

            if self.start and len(self.target_list) == 0 and self.move_count >= self.move_count_max:
                self.start = False
                self.move_step = 0
                self.get_logger().info(f'All targets have been reached')
                self.get_logger().info(f'Go to home position')
                self.get_logger().info(f'Stop time : {self.get_clock().now().to_msg()}')
                self.call_moveit_target(self.home_target)
                
                
        return response
    
    def save_yaml(self, pick=False, place=False):

        if not os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'w') as file:
                yaml.dump({}, file)

        with open(self.yaml_path, 'r') as file:
            value = yaml.safe_load(file) or {}

        if pick:
            key = f'Pick target {self.move_count}'
            value[key] = {
            'target': self.pick_target,
            'real_move': self.curent_pose
            }

        elif place:
            key = f'Place target {self.move_count}'
            value[key] = {
            'target': self.place_target,
            'real_move': self.curent_pose
            }

        with open(self.yaml_path, 'w') as file:
            yaml.dump(value, file)
    
    def call_moveit_target(self, target):
        request = MoveItTarget.Request()
        request.target.position.x = target[0]
        request.target.position.y = target[1]
        request.target.position.z = target[2]

        result = self.moveit_target_client.call(request)

        if result.success:
            # self.get_logger().info(f'Set target done: {result.success}')
            pass
        else:
            self.get_logger().error(f'Set target failed')

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

        self.move_count += 1

        self.get_logger().info(f'Move count: {self.move_count}')
        self.get_logger().info(f'Pick target: {self.pick_target}')
        self.get_logger().info(f'Place target: {self.place_target}')
        # self.get_logger().info(f'Get target success')

    def set_target_list(self):
        upper_pick_target_1 = [self.pick_target[0], self.pick_target[1], self.pick_target[2] + 0.2]
        upper_pick_target_2 = [self.pick_target[0], self.pick_target[1], self.place_target[2] + 0.2]
        upper_pick_target_3 = [self.pick_target[0], self.place_target[1], self.place_target[2] + 0.2]
        upper_place_target = [self.place_target[0], self.place_target[1], self.place_target[2] + 0.2]

        self.target_list.append(self.home_target)
        self.target_list.append(upper_pick_target_1)
        self.target_list.append(self.pick_target)
        self.target_list.append(upper_pick_target_2)
        self.target_list.append(upper_pick_target_3)
        self.target_list.append(upper_place_target)
        self.target_list.append(self.place_target)
        self.target_list.append(upper_place_target)

        # self.get_logger().info(f'Target list: {self.target_list}')
    
    def timer_loop(self):
        if self.start and len(self.target_list) > 0:
            if self.move_done:
                self.move_done = False
                self.move_step += 1
                target = self.target_list.pop(0)
                # self.get_logger().info(f'Current target: {target}')
                self.call_moveit_target(target)

        elif self.start and len(self.target_list) == 0 and self.move_count < self.move_count_max:
            self.move_step = 0
            self.call_get_target()
            self.set_target_list()         

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
