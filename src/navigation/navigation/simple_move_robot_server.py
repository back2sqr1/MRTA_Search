#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray, Quaternion
from my_robot_interfaces.action import MoveRobot


class SimpleMoveRobotServer(Node):
    """
    Simple two-step robot navigation:
    1. Rotate to face target
    2. Move forward to target
    """
    
    def __init__(self):
        super().__init__('simple_move_robot_server')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('max_linear_velocity', 0.1)
        self.declare_parameter('max_angular_velocity', 0.3)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('angular_tolerance', 0.001)
        
        # Get parameters
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.pos_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value
        
        # Robot state
        self.current_pose = None
        self.target_pose = None
        self.current_goal_handle = None
                
        # Simple state machine: "IDLE", "ROTATING", "MOVING"
        self.state = "IDLE"

        # Get pose from pose_to_gps_simulator - optimized for fastest reception
        self.pose_sub = self.create_subscription(
            PoseArray,
            f'/pose_info',
            self.pose_array_callback,
            1  # Small queue size for latest data only
        )
        

        # Command publisher
        self.cmd_pub = self.create_publisher(
            Twist, 
            f'/{self.robot_name}/cmd_vel', 
            10
        )
        
        # Action server
        self.action_server = ActionServer(
            self,
            MoveRobot,
            f'{self.robot_name}/move_robot',
            self.execute_goal,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Control timer - high frequency for accurate control
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz for precise control
        
        self.get_logger().info(f'Simple Move Robot Server started for {self.robot_name}')


    def pose_array_callback(self, msg: PoseArray):
        """Handle pose array from Gazebo bridge"""
        if len(msg.poses) > 0:
            # Find the pose of the robot if the name is robot 1-> index 1, robot 2-> index 0
            # REMEMBER: INDICES ARE REVERSED IN POSEARRAY
            index = 0 if self.robot_name == 'robot_2' else 1
            pose = msg.poses[index]
            self.current_pose = pose
            current_yaw = self.get_yaw_from_quaternion(pose.orientation)
            self.get_logger().debug(f'{self.robot_name}: Received pose from Gazebo: '
                                  f'pos=({pose.position.x:.3f}, {pose.position.y:.3f}), '
                                  f'orient=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, '
                                  f'{pose.orientation.z:.3f}, {pose.orientation.w:.3f}), '
                                  f'yaw={math.degrees(current_yaw):.1f}°')

    def goal_callback(self, goal_request):
        """Accept goal"""
        self.get_logger().info(f'Received move goal for {self.robot_name}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info(f'Goal cancelled for {self.robot_name}')
        self.stop_robot()
        return CancelResponse.ACCEPT

    def stop_robot(self):
        """Stop robot and reset state"""
        # Stop robot
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        
        # Reset state
        self.state = "IDLE"
        self.target_pose = None
        
        if hasattr(self, 'initial_distance'):
            delattr(self, 'initial_distance')
        
        self.current_goal_handle = None

    def execute_goal(self, goal_handle: ServerGoalHandle):
        """Execute simple two-step movement"""
        self.get_logger().info(f'Executing goal for {self.robot_name}')
        
        goal = goal_handle.request
        self.current_goal_handle = goal_handle
        self.target_pose = goal.target_pose
        
        # Start with rotation phase
        self.state = "ROTATING"
        
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y
        
        self.get_logger().info(
            f'{self.robot_name}: Starting simple navigation to ({target_x:.2f}, {target_y:.2f})'
        )
        self.get_logger().info(f'{self.robot_name}: Step 1 - Rotating to face target')
    
        
        # Check if goal was cancelled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = MoveRobot.Result()
            result.success = False
            result.message = "Goal was cancelled"
            self.current_goal_handle = None
            return result
        
        # Navigation completed successfully
        result = MoveRobot.Result()
        result.success = True
        result.message = "Navigation completed successfully"
        goal_handle.succeed()
        
        # Clear goal handle
        self.current_goal_handle = None
        return result

    def control_loop(self):
        """Simple control loop - rotate then move"""
        if self.state == "IDLE" or self.current_pose is None or self.target_pose is None:
            return
            
        # Check if goal was cancelled
        if (self.current_goal_handle and 
            self.current_goal_handle.is_cancel_requested):
            self.stop_robot()
            return
            
        # Get current position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Get target position
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y
        
        # Calculate angle to target
        dx = target_x - current_x
        dy = target_y - current_y
        angle_to_target = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_error = self.normalize_angle(angle_to_target - current_yaw)
        
        # Calculate distance to target
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        cmd = Twist()
        
        if self.state == "ROTATING":
            # Step 1: Rotate to face target
            if abs(angle_error) > self.angular_tolerance:
                # Rotate towards target
                if angle_error > 0:
                    cmd.angular.z = min(self.max_angular_vel, abs(angle_error))
                else:
                    cmd.angular.z = -min(self.max_angular_vel, abs(angle_error))
                print(f'Rotating: angle_error={angle_error:.2f}rad, angular_z={cmd.angular.z:.2f}')
                
            else:
                # Rotation complete, start moving
                self.state = "MOVING"
                self.get_logger().info(f'{self.robot_name}: Step 2 - Moving forward to target')
                print(f'Current pos: ({current_x:.2f}, {current_y:.2f}), target pos: ({target_x:.2f}, {target_y:.2f})')

        elif self.state == "MOVING":
            # Step 2: Move forward to target
            if distance_to_target > self.pos_tolerance:
                # Move forward at constant speed
                cmd.linear.x = min(self.max_linear_vel, distance_to_target)

                print(f'Current pos: ({current_x:.2f}, {current_y:.2f})')
            else:
                # Reached target
                self.navigation_complete()
                return
        
        # Publish command
        self.cmd_pub.publish(cmd)
        
        # Publish feedback
        if self.current_goal_handle:
            feedback = MoveRobot.Feedback()
            feedback.current_pose = self.current_pose
            feedback.distance_to_target = distance_to_target
            
            # Calculate progress percentage
            if not hasattr(self, 'initial_distance'):
                self.initial_distance = max(distance_to_target, 0.1)
            
            if self.state == "ROTATING":
                # Rotation progress (0-30%)
                rotation_progress = max(0, min(30, 30 * (1 - abs(angle_error) / math.pi)))
                feedback.percentage_complete = float(rotation_progress)
            else:
                # Movement progress (30-100%)
                movement_progress = 30 + 70 * (1 - distance_to_target / self.initial_distance)
                feedback.percentage_complete = float(max(30, min(100, movement_progress)))

            
            feedback.current_velocity = math.sqrt(cmd.linear.x**2 + cmd.angular.z**2)
            self.current_goal_handle.publish_feedback(feedback)

    def navigation_complete(self):
        """Complete navigation and stop robot"""
        # Stop robot
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        
        self.get_logger().info(f'{self.robot_name}: Navigation completed successfully!')
        
        # Clear state
        self.state = "IDLE"
        self.target_pose = None
        
        # Clear initial distance for next goal
        if hasattr(self, 'initial_distance'):
            delattr(self, 'initial_distance')
        
        # Note: goal completion is handled in execute_goal method
        # self.current_goal_handle will be cleared there

    def get_yaw_from_quaternion(self, q: Quaternion):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveRobotServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
