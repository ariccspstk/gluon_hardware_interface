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

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    pkg_share = FindPackageShare("gluon_moveit_config")

    declared_args = [
        DeclareLaunchArgument("ip_address", default_value="192.168.1.30"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
    ]

    ip_address = LaunchConfiguration("ip_address")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # --- Single source of truth for robot_description ---
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([pkg_share, "config", "gluon.urdf.xacro"]),
                " use_fake_hardware:=", use_fake_hardware,
                " ip_address:=", ip_address,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

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
            PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"]),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner])],
        )
    )
    delay_arm = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(period=3.0, actions=[arm_controller_spawner])],
        )
    )

    # --- MoveIt config, built with the SAME mappings used above ---
    moveit_config = (
        MoveItConfigsBuilder("gluon", package_name="gluon_moveit_config")
        .robot_description(mappings={"use_fake_hardware": use_fake_hardware, "ip_address": ip_address})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_launch = generate_move_group_launch(moveit_config)
    rviz_launch = generate_moveit_rviz_launch(moveit_config)
    static_tf_launch = generate_static_virtual_joint_tfs_launch(moveit_config)

    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
            controller_manager,
            delay_jsb,
            delay_arm,
            move_group_launch,
            static_tf_launch,
            rviz_launch,
        ]
    )