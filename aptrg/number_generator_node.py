import rclpy
from random import randrange

from rclpy.node import Node 
from std_msgs.msg import Int32


class NumberGeneratorNode(Node):
    def __init__(self):
        super().__init__('number_generator_node')
        self.odd_number_publisher_ = self.create_publisher(Int32, 'odd_number', 10)
        self.timer_odd_number = self.create_timer(0.5, self.odd_number_callback)

        self.even_number_publisher_ = self.create_publisher(Int32, 'even_number', 10)
        self.timer_even_number = self.create_timer(0.5, self.even_number_callback)

        self.rand_number_publisher_ = self.create_publisher(Int32, 'rand_number', 10)
        self.timer_rand_number = self.create_timer(0.5, self.rand_number_callback)
        
        self.odd_number = 1
        self.even_number = 0

    def odd_number_callback(self):
        msg = Int32()
        msg.data = self.odd_number

        self.odd_number_publisher_.publish(msg)
        self.get_logger().info(f"Publish odd number: {self.odd_number}")
        self.odd_number += 2

    def even_number_callback(self):
        msg = Int32()
        msg.data = self.even_number

        self.even_number_publisher_.publish(msg)
        self.get_logger().info(f"Publish even number: {self.even_number}")
        self.even_number += 2

    def rand_number_callback(self):
        msg = Int32()
        msg.data = randrange(100)

        self.rand_number_publisher_.publish(msg)
        self.get_logger().info(f"Publish rand number: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    number_generator_node = NumberGeneratorNode()

    rclpy.spin(number_generator_node)

    number_generator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

