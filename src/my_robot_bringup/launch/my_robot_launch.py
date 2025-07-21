from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import yaml
import os
import json

def load_config():
    """Load configuration from YAML file"""
    config_path = os.path.join(
        os.path.dirname(__file__), 
        '..', 'config', 'robot_launch_config.yaml'
    )
    config_path = os.path.abspath(config_path)
    
    try:
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Warning: Config file not found at {config_path}. Using default values.")
        return []


def parse_robot_positions(position_string):
    """Parse position string into list of [x, y, z] coordinates"""
    if not position_string:
        return [[0.0, 0.0, 0.0]]
    
    positions = []
    for pos_str in position_string.split(';'):
        if pos_str.strip():
            coords = [float(x.strip()) for x in pos_str.split(',')]
            if len(coords) == 3:
                positions.append(coords)
    print(f"Parsed robot positions: {positions}")
    return positions if positions else [[0.0, 0.0, 0.0]]

def launch_setup(context, *args, **kwargs):
    """Setup function to handle dynamic robot spawning"""
    
    # Load configuration
    config = load_config()
    
    # Get the actual values of launch configurations
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    robot_positions_str = LaunchConfiguration('robot_positions').perform(context)
    
    # Parse positions
    positions = parse_robot_positions(robot_positions_str)
    
    # If no positions provided, use default from config
    if not positions or positions == [[0.0, 0.0, 0.0]]:
        positions = config['robots']['default_positions']
    
    # Package paths from config
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        config['paths']['urdf_file']
    ])
    
    # Get output mode from config
    output_mode = config['output']['mode']
    
    # Get robot name prefix from config
    name_prefix = config['robots']['name_prefix']
    
    # Create robot groups dynamically based on num_robots
    robot_actions = []
    
    for i in range(num_robots):
        robot_name = f'{name_prefix}{i+1}'
        pos = positions[i]
        
        # Robot state publisher in global namespace to publish TF correctly
        robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ', urdf_path,
                    ' robot_name:=', robot_name,
                    ' robot_x:=', str(pos[0]),
                    ' robot_y:=', str(pos[1]),
                    ' robot_z:=', str(pos[2])
                ])
            }],
            # Remap robot_description to robot-specific topic
            remappings=[
                ('/robot_description', f'/{robot_name}/robot_description'),
                ('/joint_states', f'/{robot_name}/joint_states')
            ],
            output=output_mode
        )
        
        # Joint state publisher for wheel joints
        joint_state_pub = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=f'{robot_name}_joint_state_publisher',
            remappings=[
                ('/robot_description', f'/{robot_name}/robot_description'),
                ('/joint_states', f'/{robot_name}/joint_states')
            ],
            output=output_mode
        )
        
        # Gazebo spawn in robot namespace
        robot_spawn_group = GroupAction([
            PushRosNamespace(robot_name),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', f'/{robot_name}/robot_description',
                    '-name', robot_name,
                    '-x', str(pos[0]),
                    '-y', str(pos[1]),
                    '-z', str(pos[2])
                ],
                output=output_mode
            )
        ])
        
        robot_actions.append(robot_state_pub)
        robot_actions.append(joint_state_pub)
        robot_actions.append(robot_spawn_group)
    
    # Flatten positions array for parameter passing
    flattened_positions = []
    for i in range(min(num_robots, len(positions))):
        flattened_positions.extend(positions[i])
    
    return robot_actions

def generate_launch_description():
    # Load configuration
    config = load_config()

    with open('src/my_robot_bringup/config/bdd.json', 'r') as file:
        bdd_config = json.load(file)
    
    # Declare arguments with defaults from config
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value=str(config['robots']['num_robots']),
        description='Number of robots to spawn'
    )
    
    # Convert default positions to string format
    default_positions_str = ';'.join([
        ','.join([str(coord) for coord in pos]) 
        for pos in config['robots']['default_positions']
    ])
    
    robot_positions_arg = DeclareLaunchArgument(
        'robot_positions',
        default_value=default_positions_str,
        description='Robot positions as x,y,z;x,y,z;... format'
    )
    
    # Package paths from config
    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        config['paths']['gazebo_bridge_config']
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        config['paths']['rviz_config']
    ])
    
    # Launch Gazebo with config
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': config['gazebo']['gz_args']}.items()
    )
    
    # Use OpaqueFunction to handle dynamic robot creation
    robot_spawning = OpaqueFunction(function=launch_setup)
    
    # Spawn a marker at (0, 0, 0) - a red cylinder

    markers = []
    marker_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'marker.sdf'
            ]),
            '-name', 'origin_marker',
            '-x', '0',
            '-y', '0', 
            '-z', '0'
        ],
        output=config['output']['mode']
    )
    markers.append(marker_spawn)

    for location, pose in bdd_config['locations'].items():
        marker = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', PathJoinSubstitution([
                    FindPackageShare('my_robot_description'),
                    'urdf',
                    'marker.sdf'
                ]),
                '-name', f"marker_{location}",
                '-x', str(pose[0]),
                '-y', str(pose[1]),
                '-z', str(0)
            ],
            output=config['output']['mode']
        )
        markers.append(marker)

    # Static transform publisher for world base footprint from config
    transform_config = config['transforms']['map_to_world']
    world_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', str(transform_config['translation'][0]),
            '--y', str(transform_config['translation'][1]),
            '--z', str(transform_config['translation'][2]),
            '--roll', str(transform_config['rotation'][0]),
            '--pitch', str(transform_config['rotation'][1]),
            '--yaw', str(transform_config['rotation'][2]),
            '--frame-id', transform_config['parent_frame'],
            '--child-frame-id', transform_config['child_frame']
        ],
        output=config['output']['mode']
    )
    
    # Bridge and RViz with config
    bridge_remappings = config['remappings']['bridge']
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        remappings=[
            ('/tf', bridge_remappings['tf']),
            ('/tf_static', bridge_remappings['tf_static'])
        ],
        output=config['output']['mode']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output=config['output']['mode']
    )
    final_list = [
        num_robots_arg,
        robot_positions_arg,
        gazebo_launch,
        world_tf_publisher,
        robot_spawning,
        bridge_node,
        # rviz_node
    ]
    final_list.extend(markers)

    return LaunchDescription(final_list)