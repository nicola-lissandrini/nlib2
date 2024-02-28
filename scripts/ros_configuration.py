#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_srvs.srv import Empty

    
class RosConfiguration(Node):
    def __init__(self):
        super().__init__('ros_configuration', automatically_declare_parameters_from_overrides=True)
        self.service = self.create_service (Empty, "ros_configuration_alive", self.is_alive_callback)

    def is_alive_callback (self, request, response):
        self.get_logger().info ("Requested is alive")
        return response
        

def main(args=None):
    rclpy.init(args=args)
    node = RosConfiguration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()