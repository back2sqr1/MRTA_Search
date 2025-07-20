from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

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
    return positions if positions else [[0.0, 0.0, 0.0]]

def launch_setup(context, *args, **kwargs):
    """Setup function to handle dynamic robot spawning"""
    
    # Get the actual values of launch configurations
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    robot_positions_str = LaunchConfiguration('robot_positions').perform(context)
    
    # Parse positions
    positions = parse_robot_positions(robot_positions_str)
    
    # Ensure we have enough positions for all robots
    while len(positions) < num_robots:
        last_pos = positions[-1]
        new_pos = [last_pos[0] + 2.0, last_pos[1], last_pos[2]]
        positions.append(new_pos)
    
    # Package paths
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf', 'my_robot.urdf.xacro'
    ])
    
    # Create robot groups dynamically based on num_robots
    robot_actions = []
    
    for i in range(num_robots):
        robot_name = f'robot_{i+1}'
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
            output='screen'
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
            output='screen'
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
                output='screen'
            )
        ])
        
        robot_actions.append(robot_state_pub)
        robot_actions.append(joint_state_pub)
        robot_actions.append(robot_spawn_group)
    
    return robot_actions

def generate_launch_description():
    # Declare arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn'
    )
    
    robot_positions_arg = DeclareLaunchArgument(
        'robot_positions',
        default_value='0.0,0.0,0.0;2.0,0.0,0.0;4.0,0.0,0.0',
        description='Robot positions as x,y,z;x,y,z;... format'
    )
    
    # Package paths
    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config', 'gazebo_bridge.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'rviz', 'urdf_config.rviz'
    ])
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )
    
    # Use OpaqueFunction to handle dynamic robot creation
    robot_spawning = OpaqueFunction(function=launch_setup)
    
    # Static transform publisher for world base footprint
    world_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world_base_footprint'],
        output='screen'
    )
    
    # Static transform publisher for world base footprint
    world_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world_base_footprint'],
        output='screen'
    )
    
    # Bridge and RViz
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        num_robots_arg,
        robot_positions_arg,
        gazebo_launch,
        world_tf_publisher,
        robot_spawning,
        bridge_node,
        rviz_node
    ])