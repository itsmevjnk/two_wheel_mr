# Copyright 2020 ros2_control Development Team
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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    share_dir = get_package_share_directory('two_wheel_mr')

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("two_wheel_mr"), "rviz", "drivetrain.rviz"]
    )

    with open(os.path.join(share_dir, 'config', 'arm_kinematics.yaml')) as f:
        robot_description_kinematics = {
            'robot_description_kinematics': yaml.safe_load(f)
        }
    
    with open(os.path.join(share_dir, 'config', 'arm_joint_limits.yaml')) as f:
        robot_description_planning = {
            'robot_description_planning': yaml.safe_load(f)
        }

    planning_pipelines = {
        'planning_pipelines': [
            'ompl',
            'pilz_industrial_motion_planner',
            'chomp'
        ],
        'default_planning_pipeline': 'ompl'
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines,
            robot_description_planning
        ],
    )

    nodes = [
        rviz_node
    ]

    return LaunchDescription(nodes)