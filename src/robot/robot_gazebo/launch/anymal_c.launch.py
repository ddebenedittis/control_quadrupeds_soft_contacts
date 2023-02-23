import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    
    contact_constraint_type = LaunchConfiguration('contact_constraint_type', default="'soft_kv'")
    
    terrain_type = LaunchConfiguration('terrain_type', default='rigid')
    
    gait = LaunchConfiguration('gait', default='static_walk')
    
    save_csv = LaunchConfiguration('save_csv', default='False')
    
    use_rviz = LaunchConfiguration('use_rviz', default='False')
    
    reset = LaunchConfiguration('reset', default='False')
    
    # ======================================================================== #

    return LaunchDescription([
        
        DeclareLaunchArgument('contact_constraint_type', default_value="'soft_kv'"),

        DeclareLaunchArgument('terrain_type', default_value='rigid'),
        
        DeclareLaunchArgument('save_csv', default_value='False'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'robot.launch.py')
            ),
            launch_arguments = {
                'package_name': 'anymal_c_simple_description',
                'xacro_file_path': os.path.join('urdf', 'anymal.xacro'),
                'config_file_path': os.path.join('config', 'anymal_controller_effort.yaml'),
                'world_file_path': os.path.join('worlds', 'anymal.world'),
                'contact_constraint_type': contact_constraint_type,
                'terrain_type': terrain_type,
                'gait': gait,
                'save_csv': save_csv,
                'use_rviz': use_rviz,
            }.items(),
            condition=IfCondition(PythonExpression(['not ', reset]))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'reset_robot.launch.py')
            ),
            launch_arguments = {
                'package_name': 'anymal_c_simple_description',
                'xacro_file_path': os.path.join('urdf', 'anymal.xacro'),
                'config_file_path': os.path.join('config', 'anymal_controller_effort.yaml'),
                'world_file_path': os.path.join('worlds', 'anymal.world'),
                'contact_constraint_type': contact_constraint_type,
                'terrain_type': terrain_type,
                'gait': gait,
                'save_csv': save_csv,
                'use_rviz': use_rviz,
            }.items(),
            condition=IfCondition(reset)
        ),
    ])