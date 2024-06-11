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



import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution


def generate_launch_description():

    turtlebot4_navigation_dir = get_package_share_directory('turtlebot4_navigation')
    multi_robot_dir = get_package_share_directory('multi_robot_experiments')
    turtlebot4_viz_dir = get_package_share_directory('turtlebot4_viz')


    config = os.path.join(multi_robot_dir, 'config', 'param.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print(exc)

    # Launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(multi_robot_dir, 'map', 'lab.yaml'),
        description='Full path to map file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    robots_number = conf['multi_robot']['robots_number']

    robot_instance_cmds = []
    robots_viz = []
    for num in range(robots_number):
        rb_name = f"robot{num+1}"
        rb_params = conf['multi_robot'][rb_name]
        # if "param_file" in rb_params:
        #     params_file = os.path.join(multi_robot_dir, 'config', rb_params['param_file'])
        # else:
        #     params_file = PathJoinSubstitution([
        #                       get_package_share_directory('turtlebot4_navigation'),
        #                       'config',
        #                       'nav2.yaml'
        #                       ]
        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(turtlebot4_navigation_dir, 
                                                           'launch',
                                                           'nav2.launch.py')),
                launch_arguments={'namespace': rb_params['namespace'],
                                  'use_sim_time': use_sim_time,
                                #   'params_file': params_file,
                                  }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(turtlebot4_navigation_dir, 
                                                           'launch',
                                                           'localization.launch.py')),
                launch_arguments={'namespace': rb_params['namespace'],
                                  'use_sim_time': use_sim_time,
                                  'map': map_yaml_file,
                                  'params' : os.path.join(multi_robot_dir, 'config', f"{rb_name}_localization.yaml"),
                                  }.items()),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', rb_params['namespace']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[rb_params['namespace'], ' map yaml: ', map_yaml_file]),
            # LogInfo(
            #     condition=IfCondition(log_settings),
            #     msg=[rb_params['namespace'], ' params yaml: ', params_file]),
        ])
        robot_instance_cmds.append(group)
        
        robots_viz.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(turtlebot4_viz_dir, 
                                                           'launch',
                                                           'view_robot.launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={'namespace': rb_params['namespace']}.items())
        )


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    for robot_instance_cmd in robot_instance_cmds:
        ld.add_action(robot_instance_cmd)
    for robot_viz in robots_viz:
        ld.add_action(robot_viz)
    return ld