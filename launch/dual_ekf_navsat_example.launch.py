# Copyright 2018 Open Source Robotics Foundation, Inc.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

params_dir = PathJoinSubstitution([
                    FindPackageShare('robot_localization'),
                    "params"
                ])


params_path = PathJoinSubstitution([
                    params_dir,
                    "dual_ekf_navsat_example.yaml"
                ])

file_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_path,
        description='Full path to the ROS2 parameters file.',
    )

def generate_launch_description():
    
    os.environ['FILE_PATH'] = str(params_dir)
    
    return LaunchDescription([
        file_params_arg,
        DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),
	
    Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('odometry/filtered', 'odometry/local')]           
           ),
    Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('odometry/filtered', 'odometry/global')]
           ),           
    Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('imu', 'imu/data'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]           

           )           
])
