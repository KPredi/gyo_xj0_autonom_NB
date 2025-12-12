from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gyo_xj0_autonom_NB', executable='triangle_robot', name='triangle_robot', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='lidar', name='lidar', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='obstacles', name='obstacles', output='screen'),

    ])