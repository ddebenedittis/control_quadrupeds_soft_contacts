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
    
    save_csv = LaunchConfiguration('save_csv', default='False')
    
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
                'package_name': 'anymal_c_softfoot_q_description',
                'xacro_file_path': os.path.join('robot', 'anymal_c_softfoot_q.urdf.xacro'),
                'config_file_path': os.path.join('config', 'anymal_c_softfoot_q_controller_effort.yaml'),
                'world_file_path': os.path.join('worlds', 'anymal.world'),
                'height': '0.62',
                'contact_constraint_type': contact_constraint_type,
                'terrain_type': terrain_type,
                'save_csv': save_csv,
            }.items(),
            condition=IfCondition(PythonExpression(['not ', reset]))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'reset_robot.launch.py')
            ),
            launch_arguments = {
                'package_name': 'anymal_c_softfoot_q_description',
                'xacro_file_path': os.path.join('robot', 'anymal_c_softfoot_q.urdf.xacro'),
                'config_file_path': os.path.join('config', 'anymal_c_softfoot_q_controller_effort.yaml'),
                'world_file_path': os.path.join('worlds', 'anymal.world'),
                'height': '0.62',
                'contact_constraint_type': contact_constraint_type,
                'terrain_type': terrain_type,
                'save_csv': save_csv,
            }.items(),
            condition=IfCondition(reset)
        ),
    ])