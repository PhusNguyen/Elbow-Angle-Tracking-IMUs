import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np

# --- Quaternion Helper Functions ---
def quaternion_array(q):
    q_array = np.array([
        q.orientation.w,
        q.orientation.x,
        q.orientation.y,
        q.orientation.z    
    ], dtype=float)
    return q_array

def quartenion_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])

def quaternion_rotate(q, v):
    q = quaternion_normalize(q)
    v_q = np.array([0.0, v[0], v[1], v[2]])
    return quaternion_mult(quaternion_mult(q, v_q), quartenion_conj(q))[1:]

def quaternion_normalize(q):
    return q / np.linalg.norm(q)

# --- Angle Estimation Function ---
def angle_estimation(qA, qB):
    # Transform to world
    bone_axis = np.array([1.0, 0.0, 0.0])
    upper_world = quaternion_rotate(qA, bone_axis)
    forearm_world = quaternion_rotate(qB, bone_axis)

    # Make unit vectors
    upper_world /= np.linalg.norm(upper_world)
    forearm_world /= np.linalg.norm(forearm_world)

    # Arccos of dot product to get angle
    dot = float(np.dot(upper_world, forearm_world))
    dot = np.clip(dot, -1.0, 1.0)
    theta = np.arccos(dot)
    theta_deg = np.degrees(theta)

    return theta_deg

# --- ElbowAngleNode Class ---
class ElbowAngleNode(Node):
    def __init__(self):
        # Init
        super().__init__('angle_estimation')
        self.imu1_msg = None
        self.imu2_msg = None
        self.angle_start = None
        
        # Subscribers
        self.sub_imu1 = self.create_subscription(Imu, '/imu1/data', self.imu1_callback, 10)
        self.sub_imu2 = self.create_subscription(Imu, '/imu2/data', self.imu2_callback, 10)

        # Publisher
        self.pub_angle = self.create_publisher(Float32, '/angle', 10)
        self.get_logger().info("ElbowAngleNode started: listening to /imu1/data, /imu2/data")

    def imu1_callback(self, msg):
        self.imu1_msg = msg
        self.estimate_and_publish()

    def imu2_callback(self, msg):
        self.imu2_msg = msg
        self.estimate_and_publish()

    def estimate_and_publish(self):
        # Check if both messages have been received
        if self.imu1_msg is None or  self.imu2_msg is None:
            return
        
        # Otherwise compute angle
        qUpper = quaternion_array(self.imu1_msg)
        qForearm = quaternion_array(self.imu2_msg)

        angle_deg = angle_estimation(qUpper, qForearm)

        if self.angle_start is None:
            self.angle_start = angle_deg

        # Create message to publish
        msg = Float32()
        msg.data = float(np.abs(self.angle_start - angle_deg))  # Calibrates the angle to start at 0 degrees

        # Publish
        self.pub_angle.publish(msg)
        self.get_logger().debug(f"Published elbow angle: {angle_deg:.2f} degrees")
        
def main():
    rclpy.init()
    node = ElbowAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.distroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()