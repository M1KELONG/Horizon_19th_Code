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
            'confidence_threshold_',
            default_value='0.7',),
        DeclareLaunchArgument(
            'ob_bottom_th_in_I',
            default_value='320',),
        DeclareLaunchArgument(
            'ob_bottom_th_in_III',
            default_value='320',),
        DeclareLaunchArgument(
            'P_cnt_th',
            default_value='20',),
        DeclareLaunchArgument(
            're_tim',
            default_value='200',),
        DeclareLaunchArgument(
            're_vel',
            default_value='0.2',),
        DeclareLaunchArgument(
            'follow_linear_speed_I',
            default_value='0.5',),
        DeclareLaunchArgument(
            'follow_angular_ratio_I',
            default_value='10.0',),
        DeclareLaunchArgument(
            'follow_linear_speed_III',
            default_value='0.5',),
        DeclareLaunchArgument(
            'follow_angular_ratio_III',
            default_value='10.0',),
        DeclareLaunchArgument(
            'line_ob_x_th',
            default_value='20',),
        DeclareLaunchArgument(
            'avoid_linear_speed_I',
            default_value='0.2',),
        DeclareLaunchArgument(
            'avoid_angular_ratio_I',
            default_value='1.5',),
        DeclareLaunchArgument(
            'ob_bottom_th_I',
            default_value='290',),
        DeclareLaunchArgument(
            'tim_I',
            default_value='300',),
        DeclareLaunchArgument(
            'avoid_linear_speed_III',
            default_value='0.2',),
        DeclareLaunchArgument(
            'avoid_angular_ratio_III',
            default_value='1.5',),
        DeclareLaunchArgument(
            'ob_bottom_th_III',
            default_value='290',),
        DeclareLaunchArgument(
            'tim_III',
            default_value='300',),
        DeclareLaunchArgument(
            'temp_lim',
            default_value='20',),
        DeclareLaunchArgument(
            'modify_cnt_th',
            default_value='20',),
        DeclareLaunchArgument(
            'line_cnt_th',
            default_value='20',),
        DeclareLaunchArgument(
            'P_bottom_th',
            default_value='400',),
        DeclareLaunchArgument(
            'park_angular_ratio',
            default_value='5.5',),
        DeclareLaunchArgument(
            'stop_linear_vel',
            default_value='0.3',),
        DeclareLaunchArgument(
            'stop_linear_vel_close',
            default_value='0.5',),
        DeclareLaunchArgument(
            'stop_cnt_th',
            default_value='10',),
        DeclareLaunchArgument(
            'QR_follow_raio',
            default_value='100.0',),
        DeclareLaunchArgument(
            'QR_follow_vel',
            default_value='0.8',),
        
        Node(
            package='racing_control',
            executable='racing_control',
            output='screen',
            parameters=[
                {"pub_control_topic": "/cmd_vel"}, 
                {"confidence_threshold_": LaunchConfiguration('confidence_threshold_')},
                {"ob_bottom_th_in_I": LaunchConfiguration('ob_bottom_th_in_I')},
                {"ob_bottom_th_in_III": LaunchConfiguration('ob_bottom_th_in_III')},
                {"P_cnt_th": LaunchConfiguration('P_cnt_th')},
                {"re_tim": LaunchConfiguration('re_tim')},
                {"re_vel": LaunchConfiguration('re_vel')},
                {"follow_linear_speed_I": LaunchConfiguration('follow_linear_speed_I')},
                {"follow_angular_ratio_I": LaunchConfiguration('follow_angular_ratio_I')},
                {"follow_linear_speed_III": LaunchConfiguration('follow_linear_speed_III')},
                {"follow_angular_ratio_III": LaunchConfiguration('follow_angular_ratio_III')},
                {"line_ob_x_th": LaunchConfiguration('line_ob_x_th')},
                {"avoid_linear_speed_I": LaunchConfiguration('avoid_linear_speed_I')},
                {"avoid_angular_ratio_I": LaunchConfiguration('avoid_angular_ratio_I')},
                {"ob_bottom_th_I": LaunchConfiguration('ob_bottom_th_I')},
                {"tim_I": LaunchConfiguration('tim_I')},
                {"avoid_linear_speed_III": LaunchConfiguration('avoid_linear_speed_III')},
                {"avoid_angular_ratio_III": LaunchConfiguration('avoid_angular_ratio_III')},
                {"ob_bottom_th_III": LaunchConfiguration('ob_bottom_th_III')},
                {"tim_III": LaunchConfiguration('tim_III')},
                {"temp_lim": LaunchConfiguration('temp_lim')},
                {"modify_cnt_th": LaunchConfiguration('modify_cnt_th')},
                {"P_cnt_th": LaunchConfiguration('P_cnt_th')},
                {"line_cnt_th": LaunchConfiguration('line_cnt_th')},
                {"P_bottom_th": LaunchConfiguration('P_bottom_th')},
                {"park_angular_ratio": LaunchConfiguration('park_angular_ratio')},
                {"stop_linear_vel": LaunchConfiguration('stop_linear_vel')},
                {"stop_linear_vel_close": LaunchConfiguration('stop_linear_vel_close')},
                {"stop_cnt_th": LaunchConfiguration('stop_cnt_th')},
                {"QR_follow_raio": LaunchConfiguration('QR_follow_raio')},
                {"QR_follow_vel": LaunchConfiguration('QR_follow_vel')},
                
            ],
            # arguments=['--ros-args', '--log-level', 'warn']
        )

    
        
    ])

    # config = os.path.join(
    #     get_package_share_directory('racing_control'),
    #     'config',
    #     'control_config.yaml'
    # )
    
    # return LaunchDescription([
    #     Node(
    #         package='racing_control',
    #         node_executable='racing_control',
    #         name="control_node", 
    #         output="screen",
    #         parameters=[config]
    #     ),
    # ])