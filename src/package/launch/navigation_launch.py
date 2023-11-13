import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the TurtleBot3 navigation launch file
    tb3_navigation_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 
        'launch', 
        'navigation2.launch.py'
    )

    # Include the TurtleBot3 navigation launch description
    turtlebot3_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_navigation_launch_file),
        launch_arguments={'use_sim_time': 'True', 'map': 'my-map.yaml'}.items(),
    )

    # Define the path to the TurtleBot3 Gazebo world launch file
    tb3_gazebo_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 
        'launch', 
        'turtlebot3_world.launch.py'
    )

    # Include the TurtleBot3 Gazebo launch description
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_gazebo_launch_file)
    )

    # Define the custom 'tartabot' node
    package_move = Node(
        package='package',
        executable='move',
        name='move',
        output='screen'
    )

    # Create and return the launch description
    return LaunchDescription([
        turtlebot3_navigation2,
        turtlebot3_gazebo,
        package_move,
    ])
