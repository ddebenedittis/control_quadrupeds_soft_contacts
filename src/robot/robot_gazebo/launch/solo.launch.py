import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'robot.launch.py')
            ),
            launch_arguments = {
                'package_name': 'solo_description',
                'xacro_file_path': os.path.join('xacro', 'solo12.urdf.xacro'),
                'config_file_path': os.path.join('config', 'solo_controller_effort.yaml'),
                'world_file_path': os.path.join('worlds', 'anymal.world'),
                'height': '0.34'
            }.items()
        ),

        Node(
            package='planners_python',
            executable='planner_mjp_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='hqp_controller',
            executable='hqp_controller_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_name': 'solo12'}
            ],
            output='screen'
        ),
    ])