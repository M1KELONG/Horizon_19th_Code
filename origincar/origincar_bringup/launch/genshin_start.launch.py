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


    usb_cam_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video8',
        description='usb camera device')

    usb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_usb_cam'),
                'launch/hobot_usb_cam.launch.py')),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_pixel_format': 'mjpeg',
            'usb_zero_copy': 'False',
            'usb_video_device': LaunchConfiguration('device')
        }.items()
    )

    carema_dis_node = Node(
        package='qrcoder',
        executable='camera_dis',
        output='screen'
    )

    qrcoder_node = Node(
        package='qrcoder',
        executable='qrcoder',
        output='screen'
    )

    bgr8_codec_node = IncludeLaunchDescription( #转码nv12
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_in_format': 'jpeg',
            'codec_out_mode': 'shared_mem',
            'codec_out_format': 'nv12',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    middle_img_codec_node = IncludeLaunchDescription( #转码bgr8
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_in_format': 'jpeg',
            'codec_out_mode': 'ros',
            'codec_out_format': 'bgr8',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/middle_img'
        }.items()
    )

    # racing_track_detection_resnet18_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('racing_track_detection_resnet'),
    #             'launch/racing_track_detection_resnet.launch.py'))
    # )

    # # 巡线检测
    # line_follower_node = Node(
    #     package='originbot_linefollower',
    #     executable='follower',
    #     output='screen'
    # )

    # 避障
    # obstable_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('racing_obstacle_detection_yolo'),
    #             'launch/racing_obstacle_detection_yolo.launch.py'))
    # )

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
    
        # jpeg图片编码&发布pkg
    jpeg_compressed_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
            {"channel": 1},
            {"in_mode": "shared_mem"},
            {"in_format": "nv12"},
            {"out_mode": "ros"},
            {"out_format": "jpeg"},
            {"sub_topic": "/hbmem_img"},
            {"pub_topic": "/image_raw/compressed"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    foxglove_node = Node(
        package='foxglove_handler',
        executable='foxglove_handler',
        output='screen'
    )
    

    return LaunchDescription([
        # rosbridge_node,
        # usb_cam_device_arg,
        # usb_node,
        # # # image codec
        # carema_dis_node,
        # # middle_img_codec_node,
        # qrcoder_node,
        # # line_follower_node,
        # # camera_launch,
        # bgr8_codec_node,
        # # control_launch,
        # foxglove_node,
        # chassis_launch,
        # racing_track_detection_resnet18_launch,
        # obstable_launch
        jpeg_compressed_codec_node
    ])