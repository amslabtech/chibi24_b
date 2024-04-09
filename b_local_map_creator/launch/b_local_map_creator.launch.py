from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='b_local_map_creator',
            executable='b_local_map_creator_node',
            output='screen',
        ),
        Node(
            package='b_obstacle_detector',
            executable='b_obstacle_detector_node',
            output='screen',
        ),
        Node(
            package='chibi24_c_localizer',
            executable='chibi24_c_localizer_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
