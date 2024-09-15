import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def listener_callback(self, msg):
        twist = Twist()
        if msg.data == "turn_right":
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -1.5
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
        elif msg.data == "turn_left":
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 1.5
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
        elif msg.data == "move_forward":
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            twist.linear.x = 1.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
        elif msg.data == "move_forward":
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            twist.linear.x = -1.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
        self.publisher_.publish(twist)

        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
