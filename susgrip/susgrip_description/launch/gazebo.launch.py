from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_desc = get_package_share_directory('susgrip_description')  # Update if necessary

    # URDF file path
    urdf_file = os.path.join(pkg_desc, 'urdf', 'susgrip_2f.urdf')

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            )
        ),

        # Launch robot_state_publisher with URDF file
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
    ])