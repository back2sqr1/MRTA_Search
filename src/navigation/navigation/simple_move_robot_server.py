#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist, Pose, PoseArray, Quaternion
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
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angular_tolerance', 0.05)

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
        self.initial_distance = 0.0

        # Simple state machine: "IDLE", "ROTATING", "MOVING"
        self.state = "IDLE"

        self.pose_sub = self.create_subscription(
            PoseArray,
            f'/pose_info',
            self.pose_array_callback,
            1
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        self.action_server = ActionServer(
            self,
            MoveRobot,
            f'{self.robot_name}/move_robot',
            self.execute_goal,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(f'Simple Move Robot Server started for {self.robot_name}')

    def pose_array_callback(self, msg: PoseArray):
        try:
            if self.robot_name == 'robot_1':
                self.current_pose = msg.poses[1]
            elif self.robot_name == 'robot_2':
                self.current_pose = msg.poses[0]
        except IndexError:
            self.get_logger().warn('PoseArray message does not contain enough poses.')

    def goal_callback(self, goal_request):
        self.get_logger().info(f'✅ Received move goal for {self.robot_name}')
        self.get_logger().info(f'Target: ({goal_request.target_pose.position.x:.2f}, {goal_request.target_pose.position.y:.2f})')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'🛑 Goal cancelled for {self.robot_name}')
        if goal_handle == self.current_goal_handle:
            self.stop_robot()
        return CancelResponse.ACCEPT

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info(f"{self.robot_name}: Robot stopped.")

        self.state = "IDLE"
        self.target_pose = None
        self.current_goal_handle = None
        if hasattr(self, 'initial_distance'):
            delattr(self, 'initial_distance')


    def execute_goal(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'🚀 Executing goal for {self.robot_name}')

        if self.current_goal_handle and self.current_goal_handle.is_active:
            self.get_logger().info('Aborting previous goal.')
            self.current_goal_handle.abort()

        self.current_goal_handle = goal_handle
        self.target_pose = goal_handle.request.target_pose

        while self.current_pose is None and rclpy.ok():
            self.get_logger().warn(f"{self.robot_name}: Waiting for current pose...", throttle_duration_sec=1)
            time.sleep(0.1)

        if not self.current_goal_handle.is_active:
            return MoveRobot.Result()

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y

        self.initial_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        if self.initial_distance < self.pos_tolerance:
            self.get_logger().info("Already at the target position.")
            goal_handle.succeed()
            result = MoveRobot.Result()
            result.success = True
            result.message = "Already at target position."
            self.stop_robot()
            return result

        self.get_logger().info(f'{self.robot_name}: Current: ({current_x:.2f}, {current_y:.2f}) -> Target: ({target_x:.2f}, {target_y:.2f})')

        self.state = "ROTATING"

        result = MoveRobot.Result()
        while self.state != "IDLE" and rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info("Goal became inactive, stopping execution.")
                self.stop_robot()
                return result
            time.sleep(0.1)

        # FIX 2: Use GoalStatus.STATUS_SUCCEEDED check (already done, just confirming context)
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            result.success = True
            result.message = "Goal reached successfully."
        else:
            result.success = False
            result.message = "Goal did not succeed (aborted or canceled)."

        return result


    def control_loop(self):
        if self.state == "IDLE" or self.current_pose is None or self.current_goal_handle is None:
            return

        # Check if the goal handle is valid and active before proceeding
        if not self.current_goal_handle.is_active:
            self.get_logger().info("Current goal is no longer active. Stopping robot.")
            self.stop_robot() # Ensure the robot is stopped if the goal somehow became inactive
            return

        if self.current_goal_handle.is_cancel_requested:
            self.get_logger().info("Goal cancellation requested. Stopping robot.")
            self.current_goal_handle.canceled()
            self.stop_robot()
            return

        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        cmd_vel_msg = Twist()

        if self.state == "ROTATING":
            target_yaw = math.atan2(dy, dx)
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            angle_error = self.normalize_angle(target_yaw - current_yaw)

            if abs(angle_error) > self.angular_tolerance:
                # Apply angular velocity proportional to the error, but capped
                angular_vel_command = max(-self.max_angular_vel, min(self.max_angular_vel, 1.5 * angle_error)) # Increased gain
                cmd_vel_msg.angular.z = angular_vel_command
            else:
                cmd_vel_msg.angular.z = 0.0
                self.get_logger().info("✔️ Rotation complete. Starting forward movement.")
                self.state = "MOVING"

        elif self.state == "MOVING":
            if distance_to_goal > self.pos_tolerance:
                cmd_vel_msg.linear.x = self.max_linear_vel

                # Re-calculate target yaw and current yaw
                target_yaw = math.atan2(dy, dx)
                current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
                angle_error = self.normalize_angle(target_yaw - current_yaw)

                # Apply a small angular correction to keep facing the goal,
                # but don't let it dominate the linear motion.
                # Reduce the gain for angular correction during linear movement.
                cmd_vel_msg.angular.z = max(-self.max_angular_vel * 0.5, min(self.max_angular_vel * 0.5, 0.5 * angle_error))

            else:
                self.get_logger().info("🏁 Target reached!")
                self.current_goal_handle.succeed()
                self.stop_robot()
            
            # Publish feedback only if current_goal_handle is not None and the goal is active
            if self.current_goal_handle and self.current_goal_handle.is_active:
                feedback_msg = MoveRobot.Feedback()
                feedback_msg.current_distance = distance_to_goal
                

                self.current_goal_handle.publish_feedback(feedback_msg)


        self.cmd_pub.publish(cmd_vel_msg)

    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveRobotServer()
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Simple Move Robot Server')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()