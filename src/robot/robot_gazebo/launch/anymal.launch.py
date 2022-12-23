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
                'package_name': 'anymal_c_simple_description',
                'xacro_file_path': os.path.join('urdf', 'anymal.xacro'),
                'config_file_path': os.path.join('config', 'anymal_controller_effort.yaml'),
                'world_file_path': os.path.join('worlds', 'anymal.world'),
            }.items()
        ),
        
        # Node(
        #     package='planners_python',
        #     executable='planner_static_walk_node',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # ),
        
        # Node(
        #     package='hqp_controller',
        #     executable='hqp_controller_node',
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #         {'robot_name': 'anymal_c'}
        #     ],
        #     output='screen'
        # ),
        
        # Node(
        #     package = 'controller_manager',
        #     executable = 'spawner',
        #     arguments = ['effort_controller', '--controller-manager', '/controller_manager'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output = 'screen'
        # ),
        
        # Node(
        #     package='pose_estimator',
        #     executable='pose_estimator_node',
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #         {'robot_name': 'anymal_c'}
        #     ],
        #     emulate_tty=True,
        #     output='screen'
        # ),
        
        Node(
            package = 'controller_manager',
            executable = 'spawner',
            arguments = ['planner', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        Node(
            package = 'controller_manager',
            executable = 'spawner',
            arguments = ['whole_body_controller', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])