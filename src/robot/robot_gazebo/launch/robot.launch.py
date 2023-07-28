import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from scripts import GazeboRosPaths



def generate_launch_description():
    
    global robot_name, robot_file_path, gazebo_config_file_path
    global rviz_config_file_path, terrain_file_path, heightmap_terrain_file_path
    global multi_terrains_file_path, terrain_increasing_slope_file_path
    global world_file_path
    
    robot_name = LaunchConfiguration('robot_name', default='anymal_c')

    robot_file_path = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('package_name', default="anymal_c_simple_description")),
        LaunchConfiguration('robot_file_path', default=os.path.join('urdf', 'anymal.xacro'))
    ])
    
    gazebo_config_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'config/gazebo_params.yaml'
    ])
    
    rviz_config_file = LaunchConfiguration('rviz_config_file', default='config.rviz')
    rviz_config_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'config',
        'rviz',
        rviz_config_file
    ])
    
    terrain_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'objects/terrain.xacro'
    ])
    
    heightmap_terrain_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'objects/heightmap_terrain.xacro'
    ])
    
    multi_terrains_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'objects/multi_terrains.xacro'
    ])
    
    terrain_increasing_slope_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'objects/terrain_increasing_slope.xacro'
    ])

    world_file_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        LaunchConfiguration('world_file_path', default=os.path.join('worlds', 'anymal.world'))
    ])
    
    
    global contact_constraint_type, gait, velocity_cmd, height, save_csv, terrain, use_rviz, use_sim_time
    
    contact_constraint_type = LaunchConfiguration('contact_constraint_type', default="'soft_kv'")

    gait = LaunchConfiguration('gait', default='static_walk')

    velocity_cmd = LaunchConfiguration('velocity_cmd', default='[0., 0., 0.]')

    height = LaunchConfiguration('height', default='0.64')
        
    save_csv = LaunchConfiguration('save_csv', default='False')

    terrain = LaunchConfiguration('terrain', default='rigid')
    
    use_rviz = LaunchConfiguration('use_rviz', default='False')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    
    # ======================================================================== #

    ld =  LaunchDescription([
        
        DeclareLaunchArgument('contact_constraint_type', default_value="'soft_kv'"),
        SetParameter(name='contact_constraint_type', value=contact_constraint_type),
        
        DeclareLaunchArgument('gait', default_value='static_walk'),
        SetParameter(name='gait', value=gait),
        
        DeclareLaunchArgument('velocity_cmd', default_value='[0., 0., 0.]'),
        SetParameter(name='velocity_cmd', value=velocity_cmd),

        DeclareLaunchArgument('terrain', default_value='rigid'),
        
        DeclareLaunchArgument('save_csv', default_value='False'),
        
        DeclareLaunchArgument('use_rviz', default_value='False'),
    ])
    
    launch_generate_dae_files(ld)
        
    launch_gazebo(ld)
        
    spawn_things(ld)
    
    spawn_controllers(ld)
    
    launch_estimator(ld)
    
    launch_logger(ld)
    
    launch_rviz(ld)
        
    return ld


# ========================= Launch_generate_dae_files ======================== #

def launch_generate_dae_files(ld):
    
    global heightmap_mesh_generator
    
    heightmap_mesh_generator = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '"', ' == "heightmap"'
            ])
        ),
        package = 'robot_gazebo',
        executable = 'generate_heightmap_mesh.py',
        output = 'screen',
    )
    
    ld.add_action(heightmap_mesh_generator)


# =============================== Launch_gazebo ============================== #

def launch_gazebo(ld):
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']
    
    gazebo_server = ExecuteProcess(
        cmd=[['ros2 launch gazebo_ros gzserver.launch.py verbose:=true pause:=true world:=', world_file_path, ' params_file:=', gazebo_config_file_path]],
        additional_env={'__NV_PRIME_RENDER_OFFLOAD': '1',
                        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                        'GAZEBO_MODEL_PATH': model,
                        'GAZEBO_PLUGIN_PATH': plugin,
                        'GAZEBO_RESOURCE_PATH': media},
        shell=True,
        output = 'screen',
    )
    
    gazebo_client = ExecuteProcess(
        cmd=[['ros2 launch gazebo_ros gzclient.launch.py']],
        additional_env={'__NV_PRIME_RENDER_OFFLOAD': '1',
                        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'},
        shell=True,
        output = 'screen',
    )
    
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    

# =============================== Spawn_things =============================== #

def spawn_things(ld):
    """Spawn the robot and the terrain."""
    
    # Spawn the robot
    
    robot_rsp = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', robot_file_path, ' remove_collisions:=True']), value_type=str)},
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
    
    # Spawn the terrain
    
    terrain_rsp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '" == "rigid" or "', terrain, '" == "soft" or ',
                '"', terrain, '" == "very_soft"'
                
            ])
        ),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'ground_plane',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', terrain_file_path, ' ', 'terrain:=', terrain]), value_type=str)}
        ],
        output = 'screen',
    )
    
    heightmap_terrain_rsp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '"', ' == "heightmap"'
            ])
        ),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'ground_plane',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', heightmap_terrain_file_path, ' ', 'terrain:=', terrain]), value_type=str)}
        ],
        output = 'screen',
    )
    
    heightmap_terrain_rsp_ev_hdl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=heightmap_mesh_generator,
            on_exit=[heightmap_terrain_rsp],
        )
    )
    
    multi_terrain_rsp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '"', ' == "multi_terrains"'
            ])
        ),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'ground_plane',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', multi_terrains_file_path, ' ', 'terrain:=', terrain]), value_type=str)}
        ],
        output = 'screen',
    )
    
    terrain_increasing_slope_rsp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', terrain, '"', ' == "increasing_slope"'
            ])
        ),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'ground_plane',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', terrain_increasing_slope_file_path, ' ', 'terrain:=', terrain]), value_type=str)}
        ],
        output = 'screen',
    )
        
    spawn_terrain = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-entity', 'ground_plane',
                     '-topic', '/ground_plane/robot_description',
                     '-x', '0', '-y', '0', '-z', '0',
                     '-R', '0', '-P', '0', '-Y', '0'],
        output = 'screen',
    )
    
    robot_rsp_state_estimator = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'state_estimator',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(['xacro', ' ', robot_file_path, ' ', 'use_gazebo:=False']), value_type=str)},
            {"frame_prefix": "state_estimator/"},
        ],
        output = 'screen',
    )
    
    ld.add_action(robot_rsp)
    ld.add_action(spawn_robot)
    ld.add_action(robot_rsp_state_estimator)
    
    ld.add_action(terrain_rsp)
    ld.add_action(heightmap_terrain_rsp_ev_hdl)
    ld.add_action(multi_terrain_rsp)
    ld.add_action(terrain_increasing_slope_rsp)
    ld.add_action(spawn_terrain)
    

# ============================= Spawn_controllers ============================ #

def spawn_controllers(ld):
    spawn_joint_state_broadcaster = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    spawn_imu_sensor_broadcaster = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    
    # Spawn a different planner depending on the gait parameter.

    spawn_teleop_base = Node(
        condition=IfCondition(
            PythonExpression([
                '"', gait, '"', ' == "teleop_base"'
            ]),
        ),
        package='teleoperate_robot',
        executable='teleop_robot_base_node',
        name='teleoperate_robot',
        prefix=['xterm -fg white -bg black -e'],
        output='screen',
        parameters=[{'robot_name': robot_name}],
    )
    
    spawn_static_walk_planner = Node(
        condition=IfCondition(
            PythonExpression([
                '"', gait, '"', ' == "static_walk"'
            ]),
        ),
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['static_walk_planner', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time},],
        output='screen',
    )
    
    spawn_planner_mjp = Node(
        condition=IfCondition(
            PythonExpression([
                '"', gait, '"', ' == "walking_trot" or ', '"', gait, '"', ' == "teleop_walking_trot"'
            ]),
        ),
        package='planners_python',
        executable='planner_mjp_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'gait': gait},    
        ],
        emulate_tty=True,
        output='screen',
    )
    
    spawn_lip_walking_trot_planner = Node(
        condition=IfCondition(
            PythonExpression([
                '"', gait, '"', ' == "walking_trot" or ', '"', gait, '"', ' == "teleop_walking_trot"'
            ]),
        ),
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['lip_planner', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time},],
        emulate_tty=True,
        output='screen',
    )

    spawn_walking_trot =  ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                '"', gait, '"', ' == "walking_trot"'
            ]),
        ),
        cmd=[['ros2 topic pub -r 1 --qos-history keep_last --qos-depth 1 ',
              '--qos-durability transient_local --qos-reliability reliable ',
              '/motion_generator/simple_velocity_command ',
              'velocity_command_msgs/msg/SimpleVelocityCommand ',
              '"{velocity_forward: ', PythonExpression([velocity_cmd, '[0]']), ', ',
              'velocity_lateral: ', PythonExpression([velocity_cmd, '[1]']), ', ',
              'yaw_rate: ', PythonExpression([velocity_cmd, '[2]']), '}"']],
        shell=True,
    )

    spawn_teleop_trot = Node(
        condition=IfCondition(
            PythonExpression([
                '"', gait, '"', ' == "teleop_walking_trot"'
            ]),
        ),
        package='teleoperate_robot',
        executable='teleop_velocity_command_node',
        name='teleoperate_robot',
        prefix=['xterm -fg white -bg black -e'],
        output='screen',
        parameters=[{'robot_name': robot_name}],
    )
    
    spawn_whole_body_controller = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['whole_body_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time},],
        emulate_tty=True,
        output='screen',
    )
    
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(spawn_imu_sensor_broadcaster)
    ld.add_action(spawn_teleop_base)
    ld.add_action(spawn_static_walk_planner)
    # ld.add_action(spawn_planner_mjp)
    ld.add_action(spawn_lip_walking_trot_planner)
    ld.add_action(spawn_walking_trot)
    ld.add_action(spawn_teleop_trot)
    ld.add_action(spawn_whole_body_controller)
    
    
# ============================= Launch_estimator ============================= #

def launch_estimator(ld):
    pose_estimator = Node(
        package='pose_estimator',
        executable='pose_estimator_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_name': robot_name}
        ],
        emulate_tty=True,
        output='screen',
    )
    
    terrain_estimator = Node(
        package='terrain_estimator',
        executable='terrain_estimator_node',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        emulate_tty=True,
        output='screen',
    )
    
    ld.add_action(pose_estimator)
    ld.add_action(terrain_estimator)
    

# =============================== Launch_logger ============================== #

def launch_logger(ld):
    logger_node = Node(
        condition=IfCondition(save_csv),
        package='logger_gazebo',
        executable='logger_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_name': robot_name},
            {'gait': gait}
        ],
        emulate_tty=True,
        output='screen',
    )
    
    ld.add_action(logger_node)
    
    
# ================================ Launch_rviz =============================== #

def launch_rviz(ld):
    ground_to_base_frame_broadcaster = Node(
        condition=IfCondition(use_rviz),
        package='rviz_legged_plugins',
        executable='ground_to_base_frame_broadcaster_node.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    terrain_projector = Node(
        condition=IfCondition(use_rviz),
        package='rviz_legged_plugins',
        executable='terrain_projector_node.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file_path],
    )
    
    ld.add_action(ground_to_base_frame_broadcaster)
    ld.add_action(terrain_projector)
    ld.add_action(rviz2)
