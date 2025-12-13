from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gyo_xj0_autonom_NB', executable='triangle_robot', name='triangle_robot', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='obstacles_publisher', name='obstacles_publisher', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='path_publisher', name='path_publisher', output='screen'),

    ])