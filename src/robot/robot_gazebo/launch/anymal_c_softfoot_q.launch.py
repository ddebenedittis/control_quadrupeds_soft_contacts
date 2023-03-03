import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression



def generate_launch_description():

    contact_constraint_type = LaunchConfiguration('contact_constraint_type', default="'soft_kv'")
    
    reset = LaunchConfiguration('reset', default='False')
    
    save_csv = LaunchConfiguration('save_csv', default='False')
    
    terrain = LaunchConfiguration('terrain', default='rigid')
    
    
    # ======================================================================== #
    
    launch_arguments = {
        'robot_name': 'anymal_c_softfoot_q',
        'package_name': 'anymal_c_softfoot_q_description',
        'robot_file_path': os.path.join('robot', 'anymal_c_softfoot_q.urdf.xacro'),
        'world_file_path': os.path.join('worlds', 'anymal_softfoot.world'),
        'contact_constraint_type': contact_constraint_type,
        'height': '0.62',
        'save_csv': save_csv,
        'terrain': terrain,
    }.items()
    
    
    return LaunchDescription([
        
        DeclareLaunchArgument('contact_constraint_type', default_value="'soft_kv'"),
        DeclareLaunchArgument('terrain', default_value='rigid'),
        DeclareLaunchArgument('save_csv', default_value='False'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'robot.launch.py')
            ),
            launch_arguments = launch_arguments,
            condition = IfCondition(PythonExpression(['not ', reset]))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'reset_robot.launch.py')
            ),
            launch_arguments = launch_arguments,
            condition=IfCondition(reset)
        ),
    ])