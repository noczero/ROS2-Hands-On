import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State

class StateSubscriber(Node):
    def __init__(self):
        super().__init__('state_subscriber')
        self.subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10  # QoS profile, 10 is the depth of the message queue
        )
        self.subscription  # Prevent unused variable warning

    def state_callback(self, msg):
        self.get_logger().info(f'Connected: {msg.connected}, Mode: {msg.mode}, Armed: {msg.armed}')

def main(args=None):
    rclpy.init(args=args)
    state_subscriber = StateSubscriber()
    rclpy.spin(state_subscriber)
    state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
