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

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for robot joints and links.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "front_lidar_ip",
            default_value="192.168.58.3",
            description="IP address of front LiDAR."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "host_ip",
            default_value="192.168.58.100",
            description="IP address of host (connected to LiDAR)."
        )
    )

    # Initialize Arguments
    rviz = LaunchConfiguration("rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("two_wheel_mr"), "urdf", "drivetrain.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "prefix:=\"",
            prefix,
            "\""
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    share_dir = get_package_share_directory('two_wheel_mr')
    moveit_config = (
        MoveItConfigsBuilder('fairino5_v6_robot', package_name='two_wheel_mr')
        # robot_description omitted intentionally; it will fail anyway since config/fairino5_v6_robot.urdf doesn't exist (and we have /robot_description published by robot_state_publisher anyway)
        .robot_description_semantic(os.path.join(share_dir, 'urdf', 'fairino5_v6_robot.srdf'))
        .robot_description_kinematics(os.path.join(share_dir, 'config', 'arm_kinematics.yaml'))
        # planning_pipelines omitted since there's none
        .trajectory_execution(os.path.join(share_dir, 'config', 'moveit_controllers.yaml')) # known name so less guesswork here
        .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
        # sensors_3d omitted since there's none
        .joint_limits(os.path.join(share_dir, 'config', 'arm_joint_limits.yaml'))
        .pilz_cartesian_limits(os.path.join(share_dir, 'config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )
    # print(moveit_config.to_dict())

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("two_wheel_mr"),
            "config",
            "controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("two_wheel_mr"), "rviz", "drivetrain.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/drivetrain_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(rviz),
    )
    base_footprint_pub_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )
    # base_scan_pub_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     output="both",
    #     arguments=['0', '0', '0.255', '0', '0', '0', 'base_link', 'base_scan']
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drivetrain_controller", "--controller-manager", "/controller_manager"],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fairino5_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    host_ip = LaunchConfiguration("host_ip")
    front_lidar_ip = LaunchConfiguration("front_lidar_ip")

    front_lidar_node = Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="front_lidar",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "base_scan",
                 "sensor_ip": front_lidar_ip,
                 "host_ip": host_ip,
                 "angle_start": -1.57,
                 "angle_end": 1.57,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ]
        )

    nodes = [
        control_node,
        move_group_node,
        robot_state_pub_node,
        robot_controller_spawner,
        arm_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        base_footprint_pub_node,
        # base_scan_pub_node,
        front_lidar_node
    ]

    return LaunchDescription(declared_arguments + nodes)