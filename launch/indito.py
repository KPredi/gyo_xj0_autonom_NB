from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gyo_xj0_autonom_NB', executable='triangle_robot', name='triangle_robot', output='screen'),
        #Node(package='gyo_xj0_autonom_NB', executable='obstacle_marker', name='obstacle_marker', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='path_follower', name='path_follower', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='path_publisher', name='path_publisher', output='screen'),

    ])