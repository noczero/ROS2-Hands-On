import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class TakeoffNode(Node):

    def __init__(self):
        super().__init__('takeoff_node')

        # Create a subscriber to the MAVROS state topic
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        # Create a publisher for the local position
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Create a client for the arming service
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Create a client for the set mode service
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Initialize state
        self.current_state = State()

        # Set a timer to publish setpoints
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Wait for services to be available
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()

    def state_callback(self, msg):
        self.current_state = msg

    def timer_callback(self):
        # Create a PoseStamped message for the setpoint
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0  # Desired altitude for takeoff

        # Publish the setpoint
        self.local_pos_pub.publish(pose)

        # Check if the drone is armed and in OFFBOARD mode
        if self.current_state.mode != "OFFBOARD" or not self.current_state.armed:
            # Set OFFBOARD mode
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(set_mode_req)

            # Arm the drone
            arm_req = CommandBool.Request()
            arm_req.value = True
            self.arming_client.call_async(arm_req)

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

