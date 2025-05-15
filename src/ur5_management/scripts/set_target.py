#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from ur5_interfaces.srv import GetTarget

class SetTarget(Node):
    def __init__(self):
        super().__init__('set_target')

        # Create a service server
        self.create_service(GetTarget, '/get_target', self.get_target_callback)

        # Variablesse
        self.call_count = 0

        self.get_logger().info(f'Set Target Node has been started')

    def get_target_callback(self, request:GetTarget.Request, response:GetTarget.Response):
        self.get_logger().info(f'service request received')
        if request.call:
            self.call_count += 1

            if self.call_count == 1:
                response.pick_target.position.x = 0.5
                response.pick_target.position.y = 0.0
                response.pick_target.position.z = -0.1

                response.place_target.position.x = 0.0
                response.place_target.position.y = 0.5
                response.place_target.position.z = 0.3
            elif self.call_count == 2:
                response.pick_target.position.x = 0.5
                response.pick_target.position.y = 0.0
                response.pick_target.position.z = 0.1

                response.place_target.position.x = 0.0
                response.place_target.position.y = -0.5
                response.place_target.position.z = 0.3
                self.call_count = 0

            self.get_logger().info(f'Setting target success')
        else:
            self.get_logger().error(f'Setting target failed')
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = SetTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
