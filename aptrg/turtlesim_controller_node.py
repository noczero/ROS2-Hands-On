import rclpy
import math
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlesimControllerNode(Node):

    def __init__(self):
        super().__init__("turtlesim_controller_node")
        self.get_logger().info("TurtlesimControllerNode is running...")

        self.cmd_vel_publisher_ = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',  
            10
        )

        self.timer_cmd_vel = self.create_timer(0.2, self.cmd_vel_callback)

        self.pose_subsription_ = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_listener_callback,
            10
        )
        self.pose_subsription_

        self.pose = None

        self.target_points = [(5.5, 5.5), (7.5, 5.5), (7.5, 7.5), (5.5, 7.5), (5.5, 5.5)]

        self.target_heading = [0, 3.14/2, 3.14, -3.14/2]
        
        self.target_index = 0

        self.LINEAR_VELOCITY = 0.5

        self.STATE = 'FORWARD'

    def logger(self):
        return self.get_logger()

    def cmd_vel_callback(self):
        # main logic guidance
        current_point = (self.pose.x, self.pose.y)
        
        target_point = self.target_points[self.target_index]

        #self.logger().info(f"current point {current_point}; target_point {target_point}")

        distance = self.euclidean_distance(current_point, target_point)

        twist_msg = Twist()

        if (distance <= 0.1):
            self.logger().info("Target reached..")

            self.target_index += 1
            
            self.STATE = "YAW"

            if (self.target_index == 4):
                self.logger().info("Mission finsihed..")
                return

         

        if self.STATE == "FORWARD":
            twist_msg.linear.x = self.LINEAR_VELOCITY
        else:
            if (self.target_index != 1 and self.pose.theta < self.target_heading[self.target_index]):
                # YAW 
                twist_msg.angular.z = self.LINEAR_VELOCITY
            else:
                self.STATE = "FORWARD"

           
        self.cmd_vel_publisher_.publish(twist_msg)
        self.get_logger().info(f"Distance to target point: {distance}, taget_point: {target_point}, current_point: {current_point}, target_index: {self.target_index}, self_state: {self.STATE}, theta: {self.pose.theta}")


    
    def pose_listener_callback(self, pose):
        self.pose = pose


    def euclidean_distance(self, current_point, target_point):
        return math.sqrt((current_point[0]-target_point[0])**2 + (current_point[1] - target_point[1])**2)



def main(args=None):
    rclpy.init(args=args)

    turtlesim_controller_node = TurtlesimControllerNode()

    rclpy.spin(turtlesim_controller_node)

    turtlesim_controller_node.destory_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
