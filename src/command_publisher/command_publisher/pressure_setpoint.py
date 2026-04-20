import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class PresSetPub(Node):
    def __init__(self):
        super().__init__('pressure_setpoint_publisher')

        # Declare parameter
        self.declare_parameter('pressure_setpoint', 100)

        # Get paremater
        self.pressure_setpoint = self.get_parameter('pressure_setpoint').value

        # Create publisher
        self.publisher_ = self.create_publisher(Float32, '/pres_set', 10)

        # Create timer
        self.timer = self.create_timer(1, self.timer_callback)

        # Logging
        self.get_logger().info(f"Publish pressure setpoint: {self.pressure_setpoint} to /pres_set")

    def timer_callback(self):
        msg = Float32()
        msg.data = float(self.pressure_setpoint)

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PresSetPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
