"""
gluon_bringup.launch.py

Launches:
  - robot_state_publisher  (with ros2_control URDF)
  - controller_manager     (ros2_control)
  - joint_state_broadcaster
  - joint_trajectory_controller
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("gluon_hardware_interface")

    # ── Launch arguments ──────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument(
            "ip_address",
            default_value="0.0.0.0",
            description="Ip address for the gluon arm",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2 for visualization.",
        ),
    ]

    ip_address = LaunchConfiguration("ip_address")
    use_rviz = LaunchConfiguration("use_rviz")

    # ── Robot description (URDF via xacro) ───────────────────────────
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([pkg_share, "config", "gluon_ros2_control.urdf.xacro"]),
                " ip_address:=",
                ip_address,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ── Nodes ─────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([pkg_share, "config", "gluon_controllers.yaml"]),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn controllers after controller_manager is up
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            ],
        )
    )

    delay_jtc = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=3.0, actions=[joint_trajectory_controller_spawner]),
            ],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
            controller_manager,
            delay_jsb,
            delay_jtc,
            rviz_node,
        ]
    )