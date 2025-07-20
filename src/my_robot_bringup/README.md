# Robot Launch Configuration

This launch system uses a YAML configuration file for better readability and maintainability.

## Configuration File

The configuration is stored in `config/robot_launch_config.yaml` and includes the following sections:

### Robot Configuration (`robots`)
- `num_robots`: Default number of robots to spawn
- `default_positions`: List of default robot positions [x, y, z]
- `name_prefix`: Prefix for robot names (e.g., "robot_" creates robot_1, robot_2, etc.)
- `auto_spacing`: Spacing between auto-generated robot positions

### File Paths (`paths`)
- `urdf_file`: Path to URDF/Xacro file (relative to my_robot_description package)
- `rviz_config`: Path to RViz configuration file (relative to my_robot_description package)
- `gazebo_bridge_config`: Path to Gazebo bridge configuration (relative to my_robot_bringup package)

### Gazebo Configuration (`gazebo`)
- `gz_args`: Arguments passed to Gazebo simulator

### Transform Configuration (`transforms`)
- `map_to_world`: Static transform configuration between map and world_base_footprint frames

### Output Configuration (`output`)
- `mode`: Output mode for all nodes (screen, log, both)

### Remapping Configuration (`remappings`)
- `bridge`: Topic remappings for the Gazebo bridge node

## Usage

### Basic Launch
```bash
ros2 launch my_robot_bringup my_robot_launch.py
```

### Launch with Custom Parameters
```bash
# Launch with 3 robots
ros2 launch my_robot_bringup my_robot_launch.py num_robots:=3

# Launch with custom positions
ros2 launch my_robot_bringup my_robot_launch.py num_robots:=2 robot_positions:="1.0,1.0,0.0;3.0,2.0,0.0"
```

## Customizing Configuration

To modify the default behavior, edit `config/robot_launch_config.yaml`:

1. **Change default number of robots:**
   ```yaml
   robots:
     num_robots: 3
   ```

2. **Modify default positions:**
   ```yaml
   robots:
     default_positions:
       - [0.0, 0.0, 0.0]
       - [3.0, 0.0, 0.0]
       - [6.0, 0.0, 0.0]
   ```

3. **Change robot naming:**
   ```yaml
   robots:
     name_prefix: "explorer_"  # Creates explorer_1, explorer_2, etc.
   ```

4. **Adjust auto-spacing:**
   ```yaml
   robots:
     auto_spacing:
       x: 1.5  # Robots spaced 1.5m apart in X direction
       y: 1.0  # 1m apart in Y direction
       z: 0.0  # Same Z level
   ```

## Benefits

- **Centralized Configuration**: All parameters in one easily readable YAML file
- **Better Documentation**: Each parameter is clearly documented
- **Easier Maintenance**: No need to modify Python code for configuration changes
- **Fallback Support**: If YAML file is missing, defaults are used
- **Flexible Override**: Launch arguments still work to override YAML defaults
