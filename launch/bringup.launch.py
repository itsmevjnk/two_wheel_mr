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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            "lidar_host_ip",
            default_value="192.168.58.100",
            description="IP address of the robot computer, located on the same subnet as the LiDAR sensors."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drivetrain_port",
            default_value="/dev/ttyS0",
            description="Serial port that the drivetrain motors are connected to."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_wheel_id",
            default_value="1",
            description="ID of the left wheel motor."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_wheel_id",
            default_value="2",
            description="ID of the right wheel motor."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_ip",
            default_value="192.168.58.2",
            description="IP address of the Fairino arm controller."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "radeye_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for the RadEye radiation sensor."
        )
    )

    # Initialize Arguments
    rviz = LaunchConfiguration("rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    prefix = LaunchConfiguration("prefix")
    drivetrain_port = LaunchConfiguration("drivetrain_port")
    left_wheel_id = LaunchConfiguration("left_wheel_id")
    right_wheel_id = LaunchConfiguration("right_wheel_id")
    arm_ip = LaunchConfiguration("arm_ip")

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
            "\" ",
            "drivetrain_port:=\"",
            drivetrain_port,
            "\" ",
            "left_wheel_id:=",
            left_wheel_id,
            " ",
            "right_wheel_id:=",
            right_wheel_id,
            " ",
            "arm_ip:=\"",
            arm_ip,
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

    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
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
                 "host_ip": lidar_host_ip,
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

    radeye_port = LaunchConfiguration("radeye_port")
    radeye_node = Node(
        package="radeye_ros",
        executable="radeye_node",
        name="radeye",
        output="screen",
        parameters=[
            {
                "port": radeye_port
            }
        ]
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'astra_adv.launch.py'))
    )

    camera_link_pub_node = Node( # TODO: add this to URDF
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
        arguments=['0', '0', '0', '0', '0', '0', 'base_scan', 'camera_link']
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
        front_lidar_node,
        radeye_node,
        camera_launch,
        camera_link_pub_node
    ]

    return LaunchDescription(declared_arguments + nodes)