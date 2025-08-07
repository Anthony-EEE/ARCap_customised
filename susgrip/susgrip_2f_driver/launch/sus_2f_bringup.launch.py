#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('susgrip_description')
    sus2f_driver_package = FindPackageShare('susgrip_2f_driver')
    
    # Initialize Arguments
    sus2f_port = LaunchConfiguration("port")
    
    launch_rviz = LaunchConfiguration("launch_rviz")
    share_dir = get_package_share_directory('susgrip_description')
    urdf_file = os.path.join(share_dir, 'urdf', 'susgrip_2f.urdf')

    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
        
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [description_package, "config", "susgrip_2f_display.rviz"]
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    susgrip_2f_driver_node = Node(
        package="susgrip_2f_driver",
        executable="susgrip_2f_driver.py",
        name="susgrip_2f_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'serial_port': sus2f_port.perform(context)},
        ],
    )
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    # Delay rviz
    delay_rviz2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[rviz_node],
        )
    )
    nodes_to_start = [
        susgrip_2f_driver_node,
        robot_state_publisher_node,
        delay_rviz2_spawner,
        rviz_node,
    ]
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="susgrip_2f"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])