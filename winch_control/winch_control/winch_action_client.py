import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from winch_msgs.action import SetTarget
from threading import Lock
import ast

class WinchClient(Node):
    """ROS 2 action client for interactively sending goals to the winch_control action server."""

    def __init__(self):
        super().__init__('winch_client')
        self._lock = Lock()
        self._action_client = ActionClient(self, SetTarget, 'winch_control')
        # self._winch_joy_sub = self.create_publisher(Winch, 'winch_joy', 20)
        self.active_goals = {}  # Track active goal futures: {goal_id_str: future}
        self.get_logger().info("Winch action client initialized")
        self.get_logger().info("Enter goals as: pull use_timer duration (e.g., 'true true 5.0')")
        self.get_logger().info("Type 'quit' to exit")

    def send_goal(self, pull: bool, use_timer: bool, duration: float):
        """Send a goal to the winch_control action server."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available after waiting")
            return False

        if duration < 0:
            self.get_logger().error(f"Invalid duration: {duration} (must be non-negative)")
            return False

        goal_msg = SetTarget.Goal()
        goal_msg.pull = pull
        goal_msg.use_timer = use_timer
        goal_msg.duration = duration

        self.get_logger().info(
            f"Sending goal: pull={pull}, use_timer={use_timer}, duration={duration}s"
        )

        # Send goal asynchronously and store future
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Handle the server's response to the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        goal_id = str(goal_handle.goal_id.uuid)  # Convert UUID to string
        self.get_logger().info(f"Goal {goal_id} accepted by server")
        with self._lock:
            self.active_goals[goal_id] = future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, gid=goal_id: self.result_callback(f, gid))

    def feedback_callback(self, feedback_msg):
        """Handle feedback messages from the server."""
        goal_id = str(feedback_msg.goal_id.uuid)  # Convert UUID to string
        status = feedback_msg.feedback.status
        self.get_logger().info(f"Feedback for goal {goal_id}: status={status:.2f}")

    def result_callback(self, future, goal_id):
        """Handle the final result of the action."""
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Goal {goal_id} succeeded: error_code={result.error_code}")
        else:
            self.get_logger().error(f"Goal {goal_id} failed: error_code={result.error_code}")
        with self._lock:
            self.active_goals.pop(goal_id, None)

    def shutdown(self):
        """Clean up the action client and node."""
        self._action_client.destroy()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    winch_client = WinchClient()

    try:
        while rclpy.ok():
            # Get user input
            user_input = input("Enter goal (pull use_timer duration) or 'quit': ").strip()
            if user_input.lower() == 'quit':
                winch_client.get_logger().info("Received quit command")
                break

            # Parse input
            try:
                parts = user_input.split()
                if len(parts) != 3:
                    winch_client.get_logger().error("Invalid input: expected 'pull use_timer duration' (e.g., 'true true 5.0')")
                    continue
                pull = ast.literal_eval(parts[0].capitalize())
                use_timer = ast.literal_eval(parts[1].capitalize())
                duration = float(parts[2])
                if not isinstance(pull, bool) or not isinstance(use_timer, bool):
                    raise ValueError("pull and use_timer must be boolean")
            except (ValueError, SyntaxError) as e:
                winch_client.get_logger().error(f"Invalid input: {e}")
                continue

            # Send goal
            if not winch_client.send_goal(pull, use_timer, duration):
                winch_client.get_logger().error(f"Failed to send goal: pull={pull}, use_timer={use_timer}, duration={duration}")

            # Spin to process callbacks
            rclpy.spin_once(winch_client, timeout_sec=0.1)

    except KeyboardInterrupt:
        winch_client.get_logger().info("Shutting down client...")
    except Exception as e:
        winch_client.get_logger().error(f"Client error: {str(e)}")
    finally:
        winch_client.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()