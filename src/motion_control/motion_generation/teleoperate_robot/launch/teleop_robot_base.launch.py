from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleoperate_robot',
            namespace='teleop_robot_base1',
            executable='teleop_robot_base_node',
            name='sim'
        )
    ])