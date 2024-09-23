import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import MessageTurtleCommands

class TurtleActionClient(Node):
    def __init__(self):
        super().__init__('turtle_action_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'turtle_commands')

    def send_goal(self, command, s, angle):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg} meters')

def main(args=None):
    rclpy.init(args=args)
    action_client = TurtleActionClient()

    future = action_client.send_goal("forward", 2, 0)
    rclpy.spin_until_future_complete(action_client, future)

    future2 = action_client.send_goal("turn_right", 0, 90)
    rclpy.spin_until_future_complete(action_client, future2)

    future3 = action_client.send_goal("forward", 1, 0)
    rclpy.spin_until_future_complete(action_client, future3)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

