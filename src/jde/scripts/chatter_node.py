#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(String, 'jde', 5)
        self.timer = self.create_timer(0.2, self.callback)

    def callback(self):
        msg = String()
        msg.data = 'Hello! ROS2 is fun'
        self.publisher.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    chatter_node = Publisher()
    rclpy.spin(chatter_node)
    chatter_node.destroy_node()
    rclpy.shutdown()
