#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Start_sign(Node):
    def __init__(self):
        super().__init__('Start_sign')
        self.get_logger().info("Genshin_Start!")
        self.msg = Int32()
        self.msg.data = 8

        self.pub = self.create_publisher(Int32, '/start_sign', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)    
    start_sign = Start_sign()
    rclpy.spin(start_sign)
    start_sign.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

