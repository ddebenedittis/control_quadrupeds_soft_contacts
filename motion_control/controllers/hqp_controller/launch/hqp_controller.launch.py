from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hqp_controller',
            namespace='hqp_controller1',
            executable='hqp_controller_node',
            name='sim'
        )
    ])