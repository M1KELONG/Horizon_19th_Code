#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                    # ROS2 Python接口库
from rclpy.node import Node                     # ROS2 节点类
from sensor_msgs.msg import Image               # 图像消息类型
from std_msgs.msg import String                 # 上位机消息发布类型
from cv_bridge import CvBridge                  # ROS与OpenCV图像转换类
import cv2 as cv
import numpy as np
import time
import subprocess


class QRcoderNode(Node):

    def __init__(self, name):
        self.result = ''
        self.i = 0

        super().__init__(name)                              # ROS2节点父类初始化
        
        # self.sub = self.create_subscription(\
        #     Image, "/middle_img", self.listener_callback, 5)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.sub = self.create_subscription(\
            Image, "/image", self.listener_callback, 10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

        self.pub = self.create_publisher(String,'foxglove_case', 10)        # 创建发布者对象（消息类型、话题名、队列长度）
        # self.timer = self.create_timer(0.05, self.timer_callback)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        self.cv_bridge = CvBridge()                         # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

    def qrcoder(self,image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)        # 灰度转换
        qrcoder = cv.QRCodeDetector()                       # 实例化二维码扫描
        codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(gray)         # 解码
        result = codeinfo
        return result

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = str(result)
    #     self.pub.publish(msg)                                     # 发布话题消息
    #     self.get_logger().info('Publishing: "%s"' % msg.data)     # 输出日志信息，提示已经完成话题发布
    #     # self.i += 1
    #     # time.sleep(2)
    #     # if (self.i>=5):
    #     #     self.destroy_node()
      
    def listener_callback(self, data):
        # self.get_logger().info('QRcoder is receiving video frame!')     # 输出日志信息，提示已进入回调函数

        image = self.cv_bridge.compressed_imgmsg_to_cv2(data, 'bgr8')   # 将ROS的图像消息转化成OpenCV图像
        result = self.qrcoder(image)                                    # 二维码检测
        # result = self.qrcoder(data)                                    # 二维码检测
        msg = String()
        if ((str(result) == "ClockWise") or (str(result) == "AntiClockWise")):
            msg.data = str(result)
            self.pub.publish(msg)                                           # 发布话题消息
            self.get_logger().info('Publishing: "%s"' % msg.data)           # 输出日志信息，提示已经完成话题发布



def main(args=None):                               # ROS2节点主入口main函数

    # i = 0

    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = QRcoderNode("qrcoder")                  # 创建ROS2节点对象并进行初始化

    rclpy.spin(node)                               # 循环等待ROS2退出

    # while (i<5):
    #     rclpy.spin_once(node)                               # 循环等待ROS2退出
    #     time.sleep(1)
    #     i += 1
    
    node.destroy_node()                            # 销毁节点对象

    # subprocess.run(["ros2","run","qrcoder","camera_dis"])
        
    rclpy.shutdown()                               # 关闭ROS2 Python接口