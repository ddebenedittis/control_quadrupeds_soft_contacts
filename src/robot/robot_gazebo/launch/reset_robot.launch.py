import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    
    robot_name = LaunchConfiguration('robot_name', default='anymal_c')

    package_share_path = FindPackageShare(LaunchConfiguration('package_name', default="anymal_c_simple_description"))

    robot_file_path = PathJoinSubstitution([
        package_share_path,
        LaunchConfiguration('robot_file_path', default=os.path.join('urdf', 'anymal.xacro'))
    ])
    
    
    contact_constraint_type = LaunchConfiguration('contact_constraint_type', default="'soft_kv'")
    
    gait = LaunchConfiguration('gait', default='static_walk')
    
    height = LaunchConfiguration('height', default='0.63')
    
    save_csv = LaunchConfiguration('save_csv', default='False')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    
    # ======================================================================== #
    
    deactivate_controllers = ExecuteProcess(
        cmd=['ros2 control switch_controllers --deactivate whole_body_controller planner imu_sensor_broadcaster joint_state_broadcaster'],
        shell=True,
    )
    
    unload_controllers = ExecuteProcess(
        cmd=[
            'ros2 control unload_controller whole_body_controller', 
            '&& ros2 control unload_controller planner',
            '&& ros2 control unload_controller imu_sensor_broadcaster',
            '&& ros2 control unload_controller joint_state_broadcaster'
            ],
        shell=True,
    )
    
    pause_and_reset_sim = ExecuteProcess(
        cmd=[
            "ros2 service call /pause_physics std_srvs/srv/Empty",
            # "&& ros2 service call /reset_simulation std_srvs/srv/Empty"
        ],
        shell=True,
    )
    
    delete_robot = ExecuteProcess(
        cmd=["ros2 service call /delete_entity 'gazebo_msgs/DeleteEntity' '{name: 'anymal'}'"],
        shell=True,
    )
    
    robot_state_pub = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ' ,robot_file_path]), value_type=str)}
        ],
        output = 'screen',
    )
    
    spawn_robot = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-topic', '/robot_description',
                        '-entity', robot_name,
                        '-x', '0', '-y', '0', '-z', height,
                        '-R', '0', '-P', '0', '-Y', '0'],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    
    joint_state_broadcaster_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    imu_sensor_broadcaster_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    
    planner_spawner = Node(
        condition=IfCondition(
            PythonExpression([
                '"', gait, '"', ' == "static_walk"'
            ]),
        ),
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['planner', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time},],
    )
    
    whole_body_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['whole_body_controller', '--controller-manager', '/controller_manager'],
        parameters=[
            {"contact_constraint_type": contact_constraint_type},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    
    # ======================================================================== #

    return LaunchDescription([
        
        DeclareLaunchArgument('contact_constraint_type', default_value="'soft_kv'"),
        SetParameter(name='contact_constraint_type', value=contact_constraint_type),
        
        DeclareLaunchArgument('gait', default_value='static_walk'),
        SetParameter(name='gait', value=gait),
        
        DeclareLaunchArgument('terrain', default_value='rigid'),
        
        DeclareLaunchArgument('save_csv', default_value='False'),
        
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_robot,
                on_completion=[
                    joint_state_broadcaster_spawner,
                    imu_sensor_broadcaster_spawner,
                    planner_spawner,
                    whole_body_controller_spawner
                ]
            )
        ),
        
        Node(
            condition=IfCondition(
                PythonExpression([
                    '"', gait, '"', ' == "walking_trot"'
                ]),
            ),
            package='planners_python',
            executable='planner_mjp_node',
            parameters=[{'use_sim_time': use_sim_time}],
            emulate_tty=True,
            output='screen',
        ),
        
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=delete_robot,
                on_completion=[spawn_robot, robot_state_pub]
            )
        ),
        
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=pause_and_reset_sim,
                on_completion=[delete_robot]
            )
        ),
        
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=unload_controllers,
                on_completion=[pause_and_reset_sim]
            )
        ),
        
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=deactivate_controllers,
                on_completion=[unload_controllers]
            )
        ),
        
        deactivate_controllers,
        
        # Node(
        #     package='pose_estimator',
        #     executable='pose_estimator_node',
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #         {'robot_name': 'anymal_c'}
        #     ],
        #     emulate_tty=True,
        #     output='screen',
        # ),
        
        Node(
            condition=IfCondition(
                PythonExpression([
                    save_csv, ' == True'
                ])
            ),
            package='logger_gazebo',
            executable='logger_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_name': 'anymal_c'}
            ],
            emulate_tty=True,
            output='screen',
        ),
        
    ])