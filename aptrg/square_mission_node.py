 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class SquareWaypointNode(Node):
    def __init__(self):
        super().__init__('square_waypoint_node')
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.current_state = State()
        self.current_waypoint_index = 0

        # Define square waypoints (side length of 10 meters)
        self.waypoints = [
            (0.0, 0.0, 2.0),  # Initial position (z is altitude)
            (10.0, 0.0, 2.0), # Move east
            (10.0, 10.0, 2.0),# Move north
            (0.0, 10.0, 2.0), # Move west
            (0.0, 0.0, 2.0)   # Move south back to start
        ]

        self.pose = PoseStamped()
        self.set_waypoint(self.current_waypoint_index)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def state_callback(self, msg):
        self.current_state = msg

    def set_waypoint(self, index):
        waypoint = self.waypoints[index]
        self.pose.pose.position.x = waypoint[0]
        self.pose.pose.position.y = waypoint[1]
        self.pose.pose.position.z = waypoint[2]
        self.get_logger().info(f'Set waypoint to: {waypoint}')

    def timer_callback(self):
        if not self.current_state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            return

        if self.current_state.mode != "OFFBOARD":
            self.set_mode_client.call_async(SetMode.Request(custom_mode='OFFBOARD'))
            self.get_logger().info('Setting OFFBOARD mode')

        if not self.current_state.armed:
            self.arming_client.call_async(CommandBool.Request(value=True))
            self.get_logger().info('Arming drone')

        # Check if the drone is close to the current waypoint
        if self.is_at_waypoint():
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            self.set_waypoint(self.current_waypoint_index)

        self.local_pos_pub.publish(self.pose)

    def is_at_waypoint(self):
        # Simple distance check to see if the drone is at the current waypoint
        tolerance = 0.5  # meters
        current_pos = self.pose.pose.position
        target_pos = self.waypoints[self.current_waypoint_index]
        distance = ((current_pos.x - target_pos[0]) ** 2 +
                    (current_pos.y - target_pos[1]) ** 2 +
                    (current_pos.z - target_pos[2]) ** 2) ** 0.5
        return distance < tolerance

def main(args=None):
    rclpy.init(args=args)
    node = SquareWaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
