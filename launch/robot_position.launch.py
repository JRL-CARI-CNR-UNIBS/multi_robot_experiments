# Copyright 2024 National Council of Research of Italy (CNR) - Intelligent Robotics Lab
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



from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
import yaml
import os


def generate_launch_description():

    multi_robot_dir = get_package_share_directory('multi_robot_experiments')
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    # waypoints_path = LaunchConfiguration('waypoints_path')

    # waypoints_path_cmd = DeclareLaunchArgument(
    #     'waypoints_path',
    #     default_value=PathJoinSubstitution([multi_robot_dir, 'config', 'waypoints.yaml']),
    #     description='Path to the waypoints file')
    
    # Declare the launch arguments
    namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    
    robot_position = Node(
        package='multi_robot_experiments',
        executable='robot_position.py',
        output='screen',
        namespace=namespace,
       remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')],
    )

    waypoints_path = os.path.join(
        multi_robot_dir,
        'config',
        'waypoints.yaml'
        )
    
    with open(waypoints_path, 'r') as file:
        config_file = yaml.safe_load(file)
        print(config_file)
        waypoints_config_params = config_file['waypoints_broadcaster_node']['ros__parameters']
        print(waypoints_config_params)

    waypoints_broadcaster = Node(
        package='multi_robot_experiments',
        executable='waypoints_broadcaster_node.py',
        output='screen',
        namespace=namespace,
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')],
        parameters=[waypoints_config_params]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(namespace_cmd)
    # ld.add_action(waypoints_path_cmd)
    ld.add_action(waypoints_broadcaster)
    # ld.add_action(robot_position)

    return ld
