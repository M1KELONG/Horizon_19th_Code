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
            'END_CNT_threshold',
            default_value='10',
            ),
        DeclareLaunchArgument(
            'temp_lim',
            default_value='20',
            ),
        DeclareLaunchArgument(
            'P_bottom_th',
            default_value='400',
            ),
        DeclareLaunchArgument(
            'ob_line_x',
            default_value='20',
            ),
        DeclareLaunchArgument(
            'ob_offset',
            default_value='20',
            ),
        DeclareLaunchArgument(
            'ob_offset_III',
            default_value='30',
            ),
        DeclareLaunchArgument(
            'recover_cnt',
            default_value='20',
            ),
        


        DeclareLaunchArgument(
            'avoid_angular_ratio1_l_I_',
            default_value='20.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio2_l_I_',
            default_value='0.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio3_l_I_',
            default_value='20.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio1_r_I_',
            default_value='20.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio2_r_I_',
            default_value='0.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio3_r_I_',
            default_value='20.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio1_l_III_',
            default_value='20.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio2_l_III_',
            default_value='0.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio3_l_III_',
            default_value='20.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio1_r_III_',
            default_value='20.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio2_r_III_',
            default_value='0.0',
            ),
        DeclareLaunchArgument(
            'avoid_angular_ratio3_r_III_',
            default_value='20.0',
            ),
    


        DeclareLaunchArgument(
            'avoid_linear_speed1_l_I_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed2_l_I_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed3_l_I_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed1_r_I_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed2_r_I_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed3_r_I_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed1_l_III_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed2_l_III_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed3_l_III_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed1_r_III_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed2_r_III_',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'avoid_linear_speed3_r_III_',
            default_value='0.3',
            ),




        DeclareLaunchArgument(
            'time1_l_I_',
            default_value='200',
            ),
        DeclareLaunchArgument(
            'time2_l_I_',
            default_value='500',
            ),
        DeclareLaunchArgument(
            'time3_l_I_',
            default_value='500',
            ),
        DeclareLaunchArgument(
            'time1_r_I_',
            default_value='200',
            ),
        DeclareLaunchArgument(
            'time2_r_I_',
            default_value='500',
            ),
        DeclareLaunchArgument(
            'time3_r_I_',
            default_value='500',
            ),
        DeclareLaunchArgument(
            'time1_l_III_',
            default_value='200',
            ),
        DeclareLaunchArgument(
            'time2_l_III_',
            default_value='500',
            ),
        DeclareLaunchArgument(
            'time3_l_III_',
            default_value='500',
            ),
        DeclareLaunchArgument(
            'time1_r_III_',
            default_value='200',
            ),
        DeclareLaunchArgument(
            'time2_r_III_',
            default_value='500',
            ),
        DeclareLaunchArgument(
            'time3_r_III_',
            default_value='500',
            ),


            
        DeclareLaunchArgument(
            'follow_angular_ratio',
            default_value='-10.0',
            ),
        DeclareLaunchArgument(
            'follow_linear_speed',
            default_value='0.3',
            ),
        DeclareLaunchArgument(
            'bottom_threshold',
            default_value='400',
            ),
        DeclareLaunchArgument(
            'bottom_threshold_ob',
            default_value='320',
            ),
        DeclareLaunchArgument(
            'bottom_threshold_ob_III',
            default_value='330',
            ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            ),   
        
        DeclareLaunchArgument(
            're_vel',
            default_value='0.2',
            ),  
        DeclareLaunchArgument(
            're_tim',
            default_value='200',
            ),  

        DeclareLaunchArgument(
            'line_cnt_th',
            default_value='10',
            ),       
        DeclareLaunchArgument(
            'ob_cnt_th',
            default_value='10',
            ),       
        DeclareLaunchArgument(
            'stop_cnt_th',
            default_value='1000',
            ),      
        DeclareLaunchArgument(
            'stop_linear_vel_close',
            default_value='8',
            ),      
        DeclareLaunchArgument(
            'stop_linear_vel',
            default_value='0.2',
            ),    
        DeclareLaunchArgument(
            'QR_speed',
            default_value='0.3',
            ),    
        DeclareLaunchArgument(
            'qr_angular_ratio',
            default_value='-0.5',
            ),    
        DeclareLaunchArgument(
            'park_angular_ratio',
            default_value='0.2',
            ),    

        DeclareLaunchArgument(
            'bottom_threshold_III',
            default_value='380',
            ),    
        DeclareLaunchArgument(
            'follow_linear_speed_III',
            default_value='0.3',
            ),    


        Node(
            package='auto_control',
            executable='auto_control',
            output='screen',
            parameters=[
                {"pub_control_topic": "/cmd_vel"}, 
                {"temp_lim": LaunchConfiguration('temp_lim')},
                {"END_CNT_threshold": LaunchConfiguration('END_CNT_threshold')},
                {"P_bottom_th": LaunchConfiguration('P_bottom_th')},
                {"ob_line_x": LaunchConfiguration('ob_line_x')},
                {"ob_offset": LaunchConfiguration('ob_offset')},
                {"ob_offset_III": LaunchConfiguration('ob_offset_III')},
                {"recover_cnt": LaunchConfiguration('recover_cnt')},

                {"avoid_angular_ratio1_l_I_": LaunchConfiguration('avoid_angular_ratio1_l_I_')},
                {"avoid_angular_ratio2_l_I_": LaunchConfiguration('avoid_angular_ratio2_l_I_')},
                {"avoid_angular_ratio3_l_I_": LaunchConfiguration('avoid_angular_ratio3_l_I_')},
                {"avoid_angular_ratio1_r_I_": LaunchConfiguration('avoid_angular_ratio1_r_I_')},
                {"avoid_angular_ratio2_r_I_": LaunchConfiguration('avoid_angular_ratio2_r_I_')},
                {"avoid_angular_ratio3_r_I_": LaunchConfiguration('avoid_angular_ratio3_r_I_')},
                {"avoid_angular_ratio1_l_III_": LaunchConfiguration('avoid_angular_ratio1_l_III_')},
                {"avoid_angular_ratio2_l_III_": LaunchConfiguration('avoid_angular_ratio2_l_III_')},
                {"avoid_angular_ratio3_l_III_": LaunchConfiguration('avoid_angular_ratio3_l_III_')},
                {"avoid_angular_ratio1_r_III_": LaunchConfiguration('avoid_angular_ratio1_r_III_')},
                {"avoid_angular_ratio2_r_III_": LaunchConfiguration('avoid_angular_ratio2_r_III_')},
                {"avoid_angular_ratio3_r_III_": LaunchConfiguration('avoid_angular_ratio3_r_III_')},

                {"avoid_linear_speed1_l_I_": LaunchConfiguration('avoid_linear_speed1_l_I_')},
                {"avoid_linear_speed2_l_I_": LaunchConfiguration('avoid_linear_speed2_l_I_')},
                {"avoid_linear_speed3_l_I_": LaunchConfiguration('avoid_linear_speed3_l_I_')},
                {"avoid_linear_speed1_r_I_": LaunchConfiguration('avoid_linear_speed1_r_I_')},
                {"avoid_linear_speed2_r_I_": LaunchConfiguration('avoid_linear_speed2_r_I_')},
                {"avoid_linear_speed3_r_I_": LaunchConfiguration('avoid_linear_speed3_r_I_')},
                {"avoid_linear_speed1_l_III_": LaunchConfiguration('avoid_linear_speed1_l_III_')},
                {"avoid_linear_speed2_l_III_": LaunchConfiguration('avoid_linear_speed2_l_III_')},
                {"avoid_linear_speed3_l_III_": LaunchConfiguration('avoid_linear_speed3_l_III_')},
                {"avoid_linear_speed1_r_III_": LaunchConfiguration('avoid_linear_speed1_r_III_')},
                {"avoid_linear_speed2_r_III_": LaunchConfiguration('avoid_linear_speed2_r_III_')},
                {"avoid_linear_speed3_r_III_": LaunchConfiguration('avoid_linear_speed3_r_III_')},

                {"follow_angular_ratio": LaunchConfiguration('follow_angular_ratio')},
                {"follow_linear_speed": LaunchConfiguration('follow_linear_speed')},
                {"bottom_threshold": LaunchConfiguration('bottom_threshold')},
                {"bottom_threshold_ob": LaunchConfiguration('bottom_threshold_ob')},
                {"bottom_threshold_ob_III": LaunchConfiguration('bottom_threshold_ob_III')},
                {"confidence_threshold": LaunchConfiguration('confidence_threshold')},

                {"time1_l_I_": LaunchConfiguration('time1_l_I_')},
                {"time2_l_I_": LaunchConfiguration('time2_l_I_')},
                {"time3_l_I_": LaunchConfiguration('time3_l_I_')},
                {"time1_r_I_": LaunchConfiguration('time1_r_I_')},
                {"time2_r_I_": LaunchConfiguration('time2_r_I_')},
                {"time3_r_I_": LaunchConfiguration('time3_r_I_')},
                {"time1_l_III_": LaunchConfiguration('time1_l_III_')},
                {"time2_l_III_": LaunchConfiguration('time2_l_III_')},
                {"time3_l_III_": LaunchConfiguration('time3_l_III_')},
                {"time1_r_III_": LaunchConfiguration('time1_r_III_')},
                {"time2_r_III_": LaunchConfiguration('time2_r_III_')},
                {"time3_r_III_": LaunchConfiguration('time3_r_III_')},

                {"re_vel": LaunchConfiguration('re_vel')},
                {"re_tim": LaunchConfiguration('re_tim')},
                {"stop_cnt_th": LaunchConfiguration('stop_cnt_th')},
                {"line_cnt_th": LaunchConfiguration('line_cnt_th')}, 
                {"ob_cnt_th": LaunchConfiguration('ob_cnt_th')}, 
                {"stop_linear_vel_close": LaunchConfiguration('stop_linear_vel_close')},
                {"stop_linear_vel": LaunchConfiguration('stop_linear_vel')},
                {"QR_speed": LaunchConfiguration('QR_speed')},
                {"qr_angular_ratio": LaunchConfiguration('qr_angular_ratio')},
                {"park_angular_ratio": LaunchConfiguration('park_angular_ratio')},
                {"follow_linear_speed_III": LaunchConfiguration('follow_linear_speed_III')},
                {"bottom_threshold_III": LaunchConfiguration('bottom_threshold_III')},
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