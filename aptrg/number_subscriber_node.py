import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class NumberSubscriberNode(Node):
    def __init__(self):
        super().__init__('number_subscriber_node')

        self.even_number_subscription_ = self.create_subscription(
            Int32,
            'even_number',
            self.even_number_listener_callback,
            10
        )
   
        self.odd_number_subscription_ = self.create_subscription(
            Int32,
            'odd_number',
            self.odd_number_listener_callback,
            10
        )

        
        self.rand_number_subscription_ = self.create_subscription(
            Int32,
            'rand_number',
            self.rand_number_listener_callback,
            10
        )


        self.even_number_subscription_
        self.odd_number_subscription_
        self.rand_number_subscription_
    
    def even_number_listener_callback(self, msg):
        self.get_logger().info(f"Even Number: {msg.data}")
    
    def odd_number_listener_callback(self, msg):
        self.get_logger().info(f"Odd Number: {msg.data}")

    def rand_number_listener_callback(self, msg):
        self.get_logger().info(f"Rand Number: {msg.data}")

   
def main(args=None):
    rclpy.init(args=args)
    
    number_subscriber_node = NumberSubscriberNode()

    rclpy.spin(number_subscriber_node)

    number_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()        
