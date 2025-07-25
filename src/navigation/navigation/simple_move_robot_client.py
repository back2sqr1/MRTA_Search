import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import MoveRobot
from geometry_msgs.msg import Pose, Point, Quaternion



class SimpleMoveRobotClient(Node):
    def __init__(self):
        super().__init__('simple_move_robot_client')
        self.declare_parameter('robot_name', 'robot_1')

        
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value


        self._action_client = ActionClient(self, MoveRobot, f'{robot_name}/move_robot')
        self._goal_handle = None

    def send_goal(self, target_pose: Pose):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return
        
        goal_msg = MoveRobot.Goal()
        goal_msg.target_pose = target_pose

        
        self.get_logger().info('Sending goal to move robot')
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return
        
        self.get_logger().info('Goal accepted, waiting for result')
        self._goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.SUCCEEDED:
            self.get_logger().info('Robot moved successfully')
        else:
            self.get_logger().error(f'Robot move failed with status: {result.status}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveRobotClient()
    Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()