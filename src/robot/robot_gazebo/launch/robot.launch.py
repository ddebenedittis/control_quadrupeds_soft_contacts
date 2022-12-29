import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess 
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_share_path = FindPackageShare('anymal_c_simple_description')

    xacro_file_path = PathJoinSubstitution([
        package_share_path,
        LaunchConfiguration('xacro_file_path', default=os.path.join('urdf', 'anymal.xacro'))
    ])

    config_file_path = PathJoinSubstitution([
        package_share_path,
        LaunchConfiguration('config_file_path', default=os.path.join('config', 'anymal_controller_effort.yaml'))
    ])

    world_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        LaunchConfiguration('world_file_path', default=os.path.join('worlds', 'anymal.world'))
    ])

    height = LaunchConfiguration('height', default='0.63')

    # ======================================================================== #

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        # The additional_env are used to launch gazebo using the dedicated graphics card (which also solves the shadow bug).
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--pause', world_file_path],
            additional_env={'__NV_PRIME_RENDER_OFFLOAD': '1',
                            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'},
            output='screen'
        ),

        Node(
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            parameters = [
                {'use_sim_time': use_sim_time},
                {'robot_description': ParameterValue(Command(['xacro', ' ' ,xacro_file_path]), value_type=str)}
            ],
            output = 'screen'
        ),

        Node(
            package = 'gazebo_ros',
            executable = 'spawn_entity.py',
            arguments = ['-topic', '/robot_description',
                         '-entity', 'anymal',
                         '-x', '0', '-y', '0', '-z', height,
                         '-R', '0', '-P', '0', '-Y', '0'],
            parameters=[{'use_sim_time': use_sim_time}],
            output = 'screen'
        ),

        Node(
            package = 'controller_manager',
            executable = 'spawner',
            arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package = 'controller_manager',
            executable = 'spawner',
            arguments = ['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
            output = 'screen'
        ),
    ])