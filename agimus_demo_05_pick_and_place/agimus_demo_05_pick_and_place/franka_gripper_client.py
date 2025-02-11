from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand


class FrankaGripperClient(object):
    def __init__(self, node: Node):
        self._node = node
        self._client = ActionClient(
            self._node, GripperCommand, "/fer_gripper/gripper_action"
        )

    def send_goal(self, position: float, max_effort: float):
        """Sends a goal to the GripperCommand action server."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self._node.get_logger().info("Waiting for action server...")
        self._client.wait_for_server()

        self._node.get_logger().info(
            f"Sending goal: position={position}, max_effort={max_effort}"
        )
        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handles the response when the goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info("Goal rejected.")
            return

        self._node.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handles the final result of the action."""
        result = future.result().result
        self._node.get_logger().info(
            f"Action completed. Reached position: {result.position}"
        )

    def feedback_callback(self, feedback_msg):
        """Handles feedback from the action server."""
        self._node.get_logger().info(
            f"Feedback: Position = {feedback_msg.feedback.position}"
        )
