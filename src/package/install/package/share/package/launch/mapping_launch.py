import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the TurtleBot3 Cartographer launch file
    cartographer_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_cartographer'),
        'launch',
        'cartographer.launch.py'
    )

    # Include the TurtleBot3 Cartographer launch description
    turtlebot3_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_launch_file),
        launch_arguments={'use_sim_time': 'True'}.items(),
    )

    # Path to the TurtleBot3 Gazebo world launch file
    gazebo_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    )

    # Include the TurtleBot3 Gazebo launch description
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # Define the TurtleBot3 teleoperation node
    turtlebot3_teleop = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='turtlebot3_teleop_keyboard',
        prefix='gnome-terminal --',
        output='screen'
    )

    # Create and return the launch description
    return LaunchDescription([
        turtlebot3_cartographer,
        turtlebot3_gazebo,
        turtlebot3_teleop,
    ])

