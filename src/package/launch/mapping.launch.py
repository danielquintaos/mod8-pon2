import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define os diretórios do pacote
    turtlebot3_navigation_dir = get_package_share_directory('turtlebot3_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Configuração do SLAM Toolbox
    slam_config = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_sync.yaml')

    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
        ),

    ])

