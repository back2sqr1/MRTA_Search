#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import MoveRobot
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import yaml
import json
import os
from ament_index_python.packages import get_package_share_directory
from navigation.planning.robot_class import Robot, RobotMap
from navigation.planning.create_plan import SearchTree
import threading
import time
def get_precomputed_path(start_coords: tuple, end_coords: tuple) -> list[tuple] | None:
    """
    Returns the precomputed path as a list of waypoints (tuples)
    given start and end coordinates, including reversed paths.

    Args:
        start_coords (tuple): The (x, y) tuple for the start of the journey.
        end_coords (tuple): The (x, y) tuple for the end of the journey.

    Returns:
        list[tuple]: A list of (x, y) tuples representing the path,
                     or None if no precomputed path is found for the given
                     start and end coordinates.
    """
    # Original precomputed paths
    original_paths = {
        ((1, 1), (3, -10)): [(1, 1), (-3, -4), (3, -10)],
        ((1, 1), (4, 4)): [(1, 1), (4, 4)],
        ((1, 1), (0, 0)): [(1, 1), (0, 0)],
        ((1, 1), (-6, 7)): [(1, 1), (-2.5, 7), (-6, 7)],
        ((1, 1), (-13, 0)): [(1, 1), (-8, 1), (-13, 0)],
        ((2, 2), (4, 4)): [(2, 2), (4, 4)],
        ((2, 2), (-6, 7)): [(2, 2), (-3, 7), (-6, 7)],
        ((2, 2), (3, -10)): [(2, 2), (3, -10)],
        ((2, 2), (-13, 0)): [(2, 2), (-8, 1), (-13, 0)],
        ((2, 2), (-3, -10)): [(2, 2), (-3, -6), (-3, -10)],
        ((4, 4), (-6, 7)): [(4, 4), (-3, 7), (-6, 7)],
        ((4, 4), (3, -10)): [(4, 4), (2, 2), (3, -10)],
        ((4, 4), (-13, 0)): [(4, 4), (-13, 4), (-13, 0)],
        ((4, 4), (-3, -10)): [(4, 4), (2, 2), (-3, -6), (-3, -10)],
        ((-6, 7), (-13, 0)): [(-6, 7), (-13, 0)],
        ((-6, 7), (3, -10)): [(-6, 7), (-3, 7), (0, 0), (3, -4), (3, -10)],
        ((-6, 7), (-3, -10)): [(-6, 7), (-3, 7), (0, 0), (-3, -4), (-3, -10)],
        ((3, -10), (-3, -10)): [(3, -10), (2, -2), (-3, -4), (-3, -10)],
        ((3, -10), (-13, 0)): [(3, -10), (2, -2), (13, -2), (-13, 0)],
        ((-3, -10), (-13, 0)): [(-3, -10), (-4, -2), (13, -2), (-13, 0)]
    }

    # Create a new dictionary to store all paths, including reversed ones
    all_paths = dict(original_paths) # Start with all original paths

    # Add reversed paths
    for (start, end), path in original_paths.items():
        reversed_path = list(reversed(path)) # Create a new reversed list
        all_paths[(end, start)] = reversed_path # Add to the dictionary with (end, start) key

    return all_paths.get((start_coords, end_coords))

class CentralServerNode(Node):
    def __init__(self):
        super().__init__('central_server_node')
        
        # Action clients for each robot
        self.robot_clients = {}
        self.robot_names = ['robot_1', 'robot_2']  # Add more robots as needed
        
        # Initialize action clients
        for robot_name in self.robot_names:
            client = ActionClient(self, MoveRobot, f'{robot_name}/move_robot')
            self.robot_clients[robot_name] = client
            self.get_logger().info(f'Created action client for {robot_name}')
        
        # Planning variables
        self.sequential_plan = []
        self.plan_lock = threading.Lock() # To safely access the plan from the thread

        # Load configuration and generate plan
        self.load_config_and_plan()
        
        # Start execution after a short delay using a one-shot timer
        self.execution_timer = self.create_timer(40, self.start_execution)
    
    def load_config_and_plan(self):
        """Load robot configuration and generate movement plan"""
        try:
            # NOTE: Assumes this node is part of the 'navigation' package to find other packages
            robot_config_path = os.path.join(
                get_package_share_directory('my_robot_bringup'),
                'config',
                'robot_launch_config.yaml'
            )
            
            with open(robot_config_path, 'r') as f:
                robot_config = yaml.safe_load(f)
            
            # Create initial robot map
            search_tree = SearchTree()
            initial_robot_map = RobotMap({
                'robot_1': Robot(id='robot_1', position=robot_config['robots']['default_positions'][0][:2]),
                'robot_2': Robot(id='robot_2', position=robot_config['robots']['default_positions'][1][:2])
            })
            
            initial_resolutions = robot_config.get('initial_resolutions')
            best_plan, *_ = search_tree.get_best_plan(initial_robot_map, initial_resolutions)
            self.get_logger().info(f'Best plan found: {best_plan}')

            current_positions = {
                'robot_1': robot_config['robots']['default_positions'][0][:2],
                'robot_2': robot_config['robots']['default_positions'][1][:2]
            }
            with self.plan_lock:
                # Convert plan to sequential format
                self.get_logger().info('Generating sequential plan...')
                self.sequential_plan = []
                for robot_id, target_pose in best_plan:
                    path = get_precomputed_path(tuple(current_positions[robot_id]), target_pose)[1:]
                    current_positions[robot_id] = target_pose
                    for position in path:
                        self.sequential_plan.append((robot_id, position))
                # self.sequential_plan = best_plan
                                
                self.get_logger().info(f'✅ Generated plan with {len(self.sequential_plan)} steps')
                for i, (robot_id, position) in enumerate(self.sequential_plan):
                    self.get_logger().info(f'   Step {i}: {robot_id} -> {position}')
                
                
                
        except Exception as e:
            self.get_logger().error(f'Failed to load config and generate plan: {e}')
            self.sequential_plan = []
    
    def start_execution(self):
        """Callback to start the plan execution in a separate thread."""
        # This timer is one-shot, so cancel it to prevent it from firing again
        self.execution_timer.cancel()
        
        with self.plan_lock:
            if not self.sequential_plan:
                self.get_logger().warn('No valid plan to execute. Shutting down.')
                return
        
        self.get_logger().info("🚀 Starting plan execution in a new thread...")
        # Run the blocking execution logic in a separate thread to not block the main ROS2 thread
        execution_thread = threading.Thread(target=self.execute_plan_thread, daemon=True)
        execution_thread.start()
        
    def execute_plan_thread(self):
        """The actual plan execution logic that runs in a separate thread."""
        with self.plan_lock:
            plan_copy = list(self.sequential_plan)

        for i, (robot_id, position) in enumerate(plan_copy):
            self.get_logger().info(f"--- Executing Step {i+1}/{len(plan_copy)}: Move {robot_id} to {position} ---")
            
            client = self.robot_clients.get(robot_id)
            if client is None:
                self.get_logger().error(f"No action client found for {robot_id}! Aborting plan.")
                return

            # Wait for the action server to be available
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Action server for {robot_id} not available! Aborting plan.")
                return
            
            # Create the goal message
            goal_msg = MoveRobot.Goal()
            goal_msg.target_pose.position.x = float(position[0])
            goal_msg.target_pose.position.y = float(position[1])
            goal_msg.target_pose.orientation.w = 1.0 # Default orientation

            # Send the goal and wait for it to be accepted
            self.get_logger().info(f"Sending goal to {robot_id}...")
            send_goal_future = client.send_goal_async(goal_msg)
            
            # Wait for the future to complete (non-blocking for the executor)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error(f"Goal for {robot_id} was rejected! Aborting plan.")
                return

            self.get_logger().info(f"Goal accepted by {robot_id}. Waiting for result...")

            # Now wait for the action to complete
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            
            result_wrapper = get_result_future.result()
            
            # Check the final status of the action
            if result_wrapper.status == GoalStatus.STATUS_SUCCEEDED:
                final_result = result_wrapper.result
                if not final_result.success:
                    self.get_logger().error(f"Movement for {robot_id} failed: {final_result.message}. Aborting plan.")
                    return
                self.get_logger().info(f"✅ Step {i+1} successful for {robot_id}.")
            else:
                self.get_logger().error(f"Movement for {robot_id} did not succeed (Status: {result_wrapper.status}). Aborting plan.")
                return
        
        self.get_logger().info("🎉🎉🎉 All steps in the plan executed successfully! 🎉🎉🎉")

def main(args=None):
    rclpy.init(args=args)
    node = CentralServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()