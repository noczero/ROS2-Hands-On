import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class Px4SquareMissionNode(Node):

    def __init__(self):

        super().__init__('px4_square_mission_node')

        self.logger().info("PX4 Square Mission Node is running...")
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        self.state_sub

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Keep the last 10 messages
        )

        self.local_position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_position_callback,
            qos_profile    
        )
        self.local_position_sub
        
        self.local_position = PoseStamped()

        self.current_state = State()

        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        self.arming_client.wait_for_service()

        self.set_mode_client =  self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.set_mode_client.wait_for_service()

        self.timer = self.create_timer(0.05, self.timer_callback)
    

    def logger(self):
        return self.get_logger()

    def state_callback(self, msg):
        self.current_state = msg

    def local_position_callback(self, msg):
        self.local_position = msg
        self.logger().info(str(msg))
    
    def timer_callback(self):
        # masin logic
        pass 


def main(args=None):
    rclpy.init(args=args)
    node = Px4SquareMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
