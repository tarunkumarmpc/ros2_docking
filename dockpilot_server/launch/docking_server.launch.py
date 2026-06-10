import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('dockpilot_server')

    config_path = os.path.join(pkg_share, 'config', 'docks_server.yaml')
    dock_configs_path = os.path.join(pkg_share, 'config', 'dock_configs.yaml')

    print(f'[launch] Dock server config file path resolved as: {config_path}')
    print(f'[launch] Dock configs file path resolved as: {dock_configs_path}')

    if not os.path.isfile(config_path):
        print(f'[launch] ERROR: Dock server config file does NOT exist at path: {config_path}')
        return LaunchDescription([])

    if not os.path.isfile(dock_configs_path):
        print(f'[launch] ERROR: Dock configs file does NOT exist at path: {dock_configs_path}')
        return LaunchDescription([])

    override_params = {
        'dock_config_file': dock_configs_path
    }

    return LaunchDescription([
        Node(
            package='dockpilot_server',
            executable='docking_server',
            name='docking_server',
            output='screen',
            parameters=[
                config_path,       # your YAML file (without dock_config_file param)
                override_params    # dictionary overriding parameters
            ],
        )
    ])
