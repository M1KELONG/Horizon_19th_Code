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

import os, launch, launch_ros

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'receive_t',                    default_value='0.05',   ),
        DeclareLaunchArgument(
            'follow_angular_ratio_I',       default_value='1.0',    ),
        DeclareLaunchArgument(
            'follow_linear_speed_I',        default_value='0.3',    ),
        DeclareLaunchArgument(
            'follow_angular_ratio_III',       default_value='1.0',    ),
        DeclareLaunchArgument(
            'follow_linear_speed_III',        default_value='0.3',    ),
        DeclareLaunchArgument(
            'ob_bottom_confidence',         default_value='0.5',    ),
        DeclareLaunchArgument(
            'ob_bottom_th_I',               default_value='320',    ),
        DeclareLaunchArgument(
            'ob_bottom_th_in_I',            default_value='290',    ),
        DeclareLaunchArgument(
            'avoid_angular_ratio_I',        default_value='1.0',    ),
        DeclareLaunchArgument(
            'avoid_linear_speed_I',         default_value='0.2',    ),
        DeclareLaunchArgument(
            'modify_cnt_th_I',              default_value='50',     ),
        DeclareLaunchArgument(
            'ob_bottom_th_III',               default_value='320',    ),
        DeclareLaunchArgument(
            'ob_bottom_th_in_III',            default_value='290',    ),
        DeclareLaunchArgument(
            'avoid_angular_ratio_III',        default_value='1.0',    ),
        DeclareLaunchArgument(
            'avoid_linear_speed_III',         default_value='0.2',    ),
        DeclareLaunchArgument(
            'modify_cnt_th_III',              default_value='50',     ),
        DeclareLaunchArgument(
            'line_ob_x_th',                 default_value='20',     ),
        DeclareLaunchArgument(
            'line_cnt_th_I',                default_value='1500',   ),
        DeclareLaunchArgument(
            'sleep_time_I',                 default_value='0.3',    ),
        DeclareLaunchArgument(
            'modify_ratio_I',               default_value='0.8',    ),
        DeclareLaunchArgument(
            'line_cnt_th_III',                default_value='1500',   ),
        DeclareLaunchArgument(
            'sleep_time_III',                 default_value='0.3',    ),
        DeclareLaunchArgument(
            'modify_ratio_III',               default_value='0.8',    ),
        DeclareLaunchArgument(
            'QR_cnt_th',                    default_value='1000',   ),
        DeclareLaunchArgument(
            'clock_conf',                   default_value='0.5',    ),
        DeclareLaunchArgument(
            'anti_conf',                    default_value='0.5',    ),
        DeclareLaunchArgument(
            'P_bottom_th',                  default_value='400',    ), 
        DeclareLaunchArgument(
            'P_cnt_th',                     default_value='50',     ),    
        DeclareLaunchArgument(
            'end_cnt_th',                   default_value='50',     ),
        DeclareLaunchArgument(
            'park_s_close',                 default_value='0.3',    ),
        DeclareLaunchArgument(
            'park_s_stop',                  default_value='0.1',    ),
        DeclareLaunchArgument(
            'park_angular_ratio',           default_value='4.0',    ),
        Node(
            package='qrcoder',
            executable='racing_control',
            output='screen',
            parameters=[
                {"pub_control_topic": "/cmd_vel"}, 
                {"receive_t": LaunchConfiguration('receive_t')},
                {"follow_angular_ratio_I": LaunchConfiguration('follow_angular_ratio_I')},
                {"follow_linear_speed_I": LaunchConfiguration('follow_linear_speed_I')},
                {"follow_angular_ratio_III": LaunchConfiguration('follow_angular_ratio_III')},
                {"follow_linear_speed_III": LaunchConfiguration('follow_linear_speed_III')},
                {"ob_bottom_th_I": LaunchConfiguration('ob_bottom_th_I')},
                {"ob_bottom_th_in_I": LaunchConfiguration('ob_bottom_th_in_I')},
                {"ob_bottom_th_III": LaunchConfiguration('ob_bottom_th_III')},
                {"ob_bottom_th_in_III": LaunchConfiguration('ob_bottom_th_in_III')},
                {"ob_bottom_confidence": LaunchConfiguration('ob_bottom_confidence')},
                {"avoid_angular_ratio_I": LaunchConfiguration('avoid_angular_ratio_I')},
                {"avoid_linear_speed_I": LaunchConfiguration('avoid_linear_speed_I')},
                {"modify_cnt_th_I": LaunchConfiguration('modify_cnt_th_I')},
                {"line_cnt_th_I": LaunchConfiguration('line_cnt_th_I')},
                {"sleep_time_I": LaunchConfiguration('sleep_time_I')},
                {"modify_ratio_I": LaunchConfiguration('modify_ratio_I')},
                {"avoid_angular_ratio_III": LaunchConfiguration('avoid_angular_ratio_III')},
                {"avoid_linear_speed_III": LaunchConfiguration('avoid_linear_speed_III')},
                {"modify_cnt_th_III": LaunchConfiguration('modify_cnt_th_III')},
                {"line_cnt_th_III": LaunchConfiguration('line_cnt_th_III')},
                {"sleep_time_III": LaunchConfiguration('sleep_time_III')},
                {"modify_ratio_III": LaunchConfiguration('modify_ratio_III')},
                {"QR_cnt_th": LaunchConfiguration('QR_cnt_th')},
                {"clock_conf": LaunchConfiguration('clock_conf')},
                {"anti_conf": LaunchConfiguration('anti_conf')},
                {"P_bottom_th": LaunchConfiguration('P_bottom_th')},
                {"P_cnt_th": LaunchConfiguration('P_cnt_th')},
                {"end_cnt_th": LaunchConfiguration('end_cnt_th')},
                {"park_s_close": LaunchConfiguration('park_s_close')},
                {"park_s_stop": LaunchConfiguration('park_s_stop')},
                {"park_angular_ratio": LaunchConfiguration('park_angular_ratio')}

                ],
                    )

    
        
    ])


                