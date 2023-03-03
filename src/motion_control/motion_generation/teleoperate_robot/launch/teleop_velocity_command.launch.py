from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleoperate_robot',
            executable='teleop_velocity_command_node',
            name='teleop_velocity_command',
            prefix=['xterm -fg white -bg black -e']
        )
    ])