from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gyo_xj0_autonom_NB', executable='utvonal', name='utvonal', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='akadalyok', name='akadalyok', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='lidar', name='lidar', output='screen'),
        Node(package='gyo_xj0_autonom_NB', executable='robot', name='robot', output='screen'),
    ])