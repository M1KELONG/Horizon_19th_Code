# Copyright (c) 2022，Horizon Robotics.
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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import TextSubstitution

def generate_launch_description():

    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('origincar_bringup'),
    #             'launch/camera_start.launch.py'))
    # )


    # usb_cam_device_arg = DeclareLaunchArgument(
    #     'device',
    #     default_value='/dev/video8',
    #     description='usb camera device')

    # usb_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('hobot_usb_cam'),
    #             'launch/hobot_usb_cam.launch.py')),
    #     launch_arguments={
    #         'usb_image_width': '640',
    #         'usb_image_height': '480',
    #         'usb_pixel_format': 'mjpeg',
    #         'usb_zero_copy': 'False',
    #         'usb_video_device': LaunchConfiguration('device')
    #     }.items()
    # )



    # bgr8_codec_node = IncludeLaunchDescription( #转码nv12
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('hobot_codec'),
    #             'launch/hobot_codec.launch.py')),
    #     launch_arguments={
    #         'codec_in_mode': 'ros',
    #         'codec_in_format': 'jpeg',
    #         'codec_out_mode': 'shared_mem',
    #         'codec_out_format': 'nv12',
    #         'codec_sub_topic': '/image',
    #         'codec_pub_topic': '/hbmem_img'
    #     }.items()
    # )

    yolotry_launch = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dnn_node_example'),
                'launch/dnn_node_example.launch.py')),
        launch_arguments={
            'dnn_example_config_file': '/root/dev_ws/src/origincar/yolotry1/yolov5workconfig.json',
            'dnn_example_image_width': '640',
            'dnn_example_image_height': '480',
        }.items()
    )

    
    # # 主控
    # control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('racing_control'),
    #             'launch/racing_control.launch.py'))
    # )

    # 底盘
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('origincar_base'),
                'launch/origincar_bringup.launch.py'))
    )

    # 上位机通讯
    rosbridge_node = Node(
        package='qrcoder',
        executable='rosbridge_node',
        output='screen'
    )

    camera_dis_node = Node(
        package='qrcoder',
        executable='camera_dis',
        output='screen'
    )
    

    return LaunchDescription([
        rosbridge_node,
        # usb_cam_device_arg,
        # usb_node,
        yolotry_launch,
        # # image codec
        # middle_img_codec_node,
        # line_follower_node,
        # camera_launch,
        # bgr8_codec_node,
        # control_launch,
        # foxglove_node,
        camera_dis_node,
        chassis_launch,
    
    ])