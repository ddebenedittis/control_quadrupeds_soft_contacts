from http.server import executable
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess 
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    package_name = 'anymal_c_simple_description'

    package_share_path = get_package_share_directory(package_name)

    xacro_file_path = os.path.join(package_share_path, 'urdf', 'anymal.xacro')

    config_file_path = os.path.join(get_package_share_directory('robot_control'), 'config', 'anymal_controller_effort')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--pause'],
            additional_env={'__NV_PRIME_RENDER_OFFLOAD': '1',
                            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'},
            output='screen'
        ),

        Node(
            package = 'controller_manager',
            executable = 'ros2_control_node',
            parameters = [
                {'robot_description': ParameterValue(Command(['xacro', ' ' ,xacro_file_path, ' gazebo:=False']), value_type=str)},
                config_file_path
            ],
            output="both",
        ),

        Node(
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            parameters = [
                {'use_sim_time': use_sim_time},
                {'robot_description': ParameterValue(Command(['xacro', ' ' ,xacro_file_path, ' gazebo:=False']), value_type=str)}
            ],
            output = 'screen'
        ),

        Node(
            package = 'gazebo_ros',
            executable = 'spawn_entity.py',
            arguments = ['-topic', '/robot_description',
                         '-entity', 'anymal',
                         '-x', '0', '-y', '0', '-z', '0.63',
                         '-R', '0', '-P', '0', '-Y', '0'],
            output = 'screen'
        ),

        Node(
            package = 'controller_manager',
            executable = 'spawner.py',
            arguments = ['effort_controllers', '-c', '/controller_manager'],
            output = 'screen'
        ),

        Node(
            package = 'controller_manager',
            executable = 'spawner.py',
            arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),

        Node(
            package = 'controller_manager',
            executable = 'spawner.py',
            arguments = ['imu_sensor_broadcaster', '-c', '/controller_manager'],
            output = 'screen'
        ),
    ])