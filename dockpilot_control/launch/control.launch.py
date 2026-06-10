from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('dockpilot_control')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'control_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_type',
            default_value='pid',
            description='Controller: pid or pure_pursuit'
        ),

        Node(
            package='dockpilot_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                {'controller_type': LaunchConfiguration('controller_type')},
                params_file
            ],
            remappings=[
                ('goal_pose',   '/goal_pose'),
                ('marker_pose', '/marker_pose'),
                ('docking_active', '/docking_active'),
                ('cmd_vel',     '/cmd_vel')
            ]
        )
    ])