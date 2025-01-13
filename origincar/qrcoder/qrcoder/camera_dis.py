#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

class Camera_dis(Node):
    def __init__(self):
        super().__init__('Camera_display')
        self.get_logger().info("Display the image on foxglove")

        # self.start_sub = self.create_subscription(Int32, '/start_sign', self.start_callback, 5)

        self.cv_bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 5)
        self.pub = self.create_publisher(CompressedImage, '/camera/display', 10)
        self.timer = self.create_timer(0.035, self.timer_callback)
        self.image = []

    # def start_callback(self,msg):
    #     if msg.data == 8:
    #         self.get_logger().info("Received start sign!")

    def image_callback(self, msg):
        # image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        xmin,ymin,w,h = 60,330,520,150
        imgcrop = image[ymin:ymin+h,xmin:xmin+w]
        image_dis = cv2.resize(imgcrop,(90,30))
        self.image = image_dis

    def timer_callback(self):
        if self.image != []:
            self.pub.publish(self.cv_bridge.cv2_to_compressed_imgmsg(self.image))

def main(args=None):
    rclpy.init(args=args)    
    camera_dis = Camera_dis()
    rclpy.spin(camera_dis)
    camera_dis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import rclpy
# import cv2
# import cv_bridge
# import numpy as np
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import CompressedImage
# from threading import Thread, Lock
# import time

# class Camera_dis(Node):
#     def __init__(self):
#         super().__init__('Camera_display')
#         self.get_logger().info("Display the image on foxglove")

#         self.cv_bridge = cv_bridge.CvBridge()
#         self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 5)
#         self.pub = self.create_publisher(CompressedImage, '/camera/display', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)  # Increased interval to 0.1 seconds

#         self.image = None
#         self.image_lock = Lock()

#         self.processing_thread = Thread(target=self.processing_loop)
#         self.processing_thread.start()

#     def image_callback(self, msg):
#         with self.image_lock:
#             self.image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

#     def processing_loop(self):
#         while rclpy.ok():
#             with self.image_lock:
#                 if self.image is not None:
#                     xmin, ymin, w, h = 60, 330, 520, 150
#                     imgcrop = self.image[ymin:ymin+h, xmin:xmin+w]
#                     image_dis = cv2.resize(imgcrop, (90, 30))
#                     compressed_imgmsg = self.cv_bridge.cv2_to_compressed_imgmsg(image_dis, 'jpg')
#                     self.pub.publish(compressed_imgmsg)
#             # time.sleep(0.1)  # Sleep to reduce CPU load

#     def timer_callback(self):
#         pass  # Removed code from timer callback to reduce load

# def main(args=None):
#     rclpy.init(args=args)
#     camera_dis = Camera_dis()
#     rclpy.spin(camera_dis)
#     camera_dis.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



