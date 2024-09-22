import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math as m
import sys

class MoveToGoalService(Node):

    def __init__(self, goal_x, goal_y):
        super().__init__('move_to_goal_service')

        self.subscriber_ = self.create_subscription(Pose, "/turtle1/pose",
                                                    self.pose_callback,
                                                     10)
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.cur_pose_ = Pose()
        self.status = 0
        self.goal_pose = Pose()
        self.goal_pose.x = goal_x
        self.goal_pose.y = goal_y

    def pose_callback(self, msg):
        self.cur_pose_ = msg
        self.raw_coords_callback(self.goal_pose)
        
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return ((goal_pose.x - self.cur_pose_.x)**2 +
                (goal_pose.y - self.cur_pose_.y)**2)**0.5

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
    
    def steering_angle(self, goal_pose):
       """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
       return m.atan2(goal_pose.y - self.cur_pose_.y, goal_pose.x - self.cur_pose_.x)

    def angular_vel(self, goal_pose, constant=6):
       """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
       return constant * (self.steering_angle(goal_pose) - self.cur_pose_.theta)

    def raw_coords_callback(self, goal_pose):
        
        tolerance = 0.2
        vel_msg = Twist()
        
        if self.euclidean_distance(goal_pose) >= tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.publisher_.publish(vel_msg)
            
        else:
            vel_msg.linear.x = 0.
            vel_msg.angular.z = 0.
            self.publisher_.publish(vel_msg)
            self.get_logger().info("Goal reached!")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run move_to_goal move_to_goal_service x y")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])

    minimal_service = MoveToGoalService(goal_x, goal_y)
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()


