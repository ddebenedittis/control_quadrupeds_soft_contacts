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
    
    use_rviz = LaunchConfiguration('use_rviz', default='False')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default="mulinex_config.rviz")

    
    # ======================================================================== #
    
    launch_arguments = {
        'robot_name': 'mulinex',
        'package_name': 'mulinex_description',
        'robot_file_path': os.path.join('urdf', 'mulinex.xacro'),
        'world_file_path': os.path.join('worlds', 'solo.world'),
        'contact_constraint_type': contact_constraint_type,
        'height': '0.05',
        'save_csv': save_csv,
        'terrain': terrain,
        'use_rviz': use_rviz,
        'rviz_config_file': rviz_config_file,
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
            condition=IfCondition(PythonExpression(['not ', reset]))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'reset_robot.launch.py')
            ),
            launch_arguments = launch_arguments,
            condition=IfCondition(reset)
        ),
    ])