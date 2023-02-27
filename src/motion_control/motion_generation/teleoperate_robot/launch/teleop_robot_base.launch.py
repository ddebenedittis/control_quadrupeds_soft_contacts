from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleoperate_robot',
            executable='teleop_robot_base_node',
            name='teleoperate_robot',
            prefix=['xterm -fg white -bg black -e']
        )
    ])