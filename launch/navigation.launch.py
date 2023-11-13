import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Define os diretórios do pacote
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_config_dir = os.path.join(nav2_bringup_dir, 'config')

    # Configurações de Navegação
    nav2_params = os.path.join(nav2_config_dir, 'nav2_params.yaml')

    return LaunchDescription([
        # Inicia a stack de navegação
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params}.items(),
        ),

    ])

