from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='b_obstacle_detector',
            executable='b_obstacle_detector_node',
            #output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='b_local_map_creator',
            executable='b_local_map_creator_node',
            #output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='chibi24_c_localizer',
            executable='chibi24_c_localizer_node',
            #output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='chibi24_c_global_path_planner',
            executable='chibi24_c_global_path_planner_node',
            #output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='b_local_goal_creator',
            executable='b_local_goal_creator_node',
            #output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='b_local_path_planner',
            executable='b_local_path_planner_node',
            #output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
    ])