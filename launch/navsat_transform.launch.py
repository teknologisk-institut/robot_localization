# Copyright 2019 Samsung Research America
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

params_path = PathJoinSubstitution([
                    FindPackageShare('robot_localization'),
                    "params",
                    "navsat_transform.yaml"
                ])

file_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_path,
        description='Full path to the ROS2 parameters file.',
    )

def generate_launch_description():
    return LaunchDescription([
        file_params_arg,
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
           )
])
