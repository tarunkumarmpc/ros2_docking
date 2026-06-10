from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dockpilot_perception'),
        'config',
        'perception.yaml'
    )

    return LaunchDescription([
        # AprilTag detector (unchanged)
        Node(
            package='dockpilot_perception',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            output='screen'),

        # Parameterised perception node
        Node(
            package='dockpilot_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[config])
    ])