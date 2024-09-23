
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from action_tutorials_interfaces.action import MessageTurtleCommands
import time

class TurtleActionServer(Node):
    def __init__(self):
        super().__init__('turtle_action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'turtle_commands',
            self.execute_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = Pose()
        self.start_pose = Pose()
        self.odom = 0

    def pose_callback(self, msg):
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.start_pose = self.current_pose
        self.odom = 0

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle

        twist = Twist()
        feedback_msg = MessageTurtleCommands.Feedback()
        result = MessageTurtleCommands.Result()

        if command == "forward":
            twist.linear.x = 1.0
            while self.odom < s:
                self.cmd_vel_pub.publish(twist)
                # self.odom = ((self.current_pose.x - self.start_pose.x)**2 + (self.current_pose.y - self.start_pose.y)**2)**0.5
                self.odom = (abs(self.current_pose.x - self.start_pose.x) + abs(self.current_pose.y - self.start_pose.y)) 
                feedback_msg.odom = round(self.odom)
                # self.get_logger().info(f"sending feedback: {feedback_msg.odom}")
                # goal_handle.publish_feedback(feedback_msg.odom)
                rclpy.spin_once(self)
                time.sleep(0.1)  # Добавьте небольшую задержку
            feedback_msg.odom = round(self.odom) + 1
            self.get_logger().info(f"sending feedback: {feedback_msg.odom}")
            # goal_handle.publish_feedback(feedback_msg)
            # rclpy.spin_once(self)
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)

        elif command == "turn_right":
            twist.angular.z = -1.0
            target_angle = self.start_pose.theta - angle * 3.14159 / 180.0
            while abs(self.current_pose.theta - target_angle) > 0.1:
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self)
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        elif command == "turn_left":
            twist.angular.z = 1.0
            target_angle = self.start_pose.theta - angle * 3.14159 / 180.0
            while abs(self.current_pose.theta - target_angle) > 0.1:
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self)
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        result.result = True
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    turtle_action_server = TurtleActionServer()
    rclpy.spin(turtle_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


