#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.publisher = self.create_subscription(String, 'jde', self.callback, 5)

    def callback(self, msg):
        print(msg.data)


if __name__ == '__main__':
    rclpy.init()
    listener_node = Subscriber()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()
