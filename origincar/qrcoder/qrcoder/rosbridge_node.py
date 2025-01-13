#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import subprocess




class ROSBridge(Node):

    def __init__(self):
        super().__init__('rosbridge_node')
        self.get_logger().info("ROS_bridge started.")
        subprocess.run(["ros2","launch","rosbridge_server","rosbridge_websocket_launch.xml"])


def main(args=None):
    # 常规写法
    rclpy.init(args=args)    
    rosbridge_node = ROSBridge()
    rclpy.spin(rosbridge_node)
    rosbridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
