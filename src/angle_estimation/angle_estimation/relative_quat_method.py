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

def quaternion_conj(q):
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
    return quaternion_mult(quaternion_mult(q, v_q), quaternion_conj(q))[1:]

def quaternion_normalize(q):
    return q / np.linalg.norm(q)

# --- Angle Calculation Methods ---
def relative_quat_method(qA, qB):
    # Relatie rotation from upper arm to forearm
    q_rel = quaternion_mult(quaternion_conj(qA), qB)
    q_rel = quaternion_normalize(q_rel)

    # --- Manual Alignment Sensor-to-Segment Calibration --- #
    # Define flexion axis (the axis the elbow rotates around)
    flexion_axis = np.array([0.0, 0.0, 1.0])

    # Project the quaternion's vector part onto the flexion axis
    # This extracts only the rotation component around that axis
    projection = np.dot(q_rel[1:], flexion_axis)

    # Reconstruct a quaternion that only rotates around the flexion axis
    angle = 2.0 * np.arctan2(abs(projection), q_rel[0])

    # Sign from the projection gives direction (flexion vs extension)
    if projection < 0:
        angle = -angle

    # # --- Functional Sensor-to-Segment Calibration --- #
    # # Convert to axis-angle
    # # The angle is the total rotation between the two segments
    # angle = 2.0 * np.arccos(np.clip(abs(q_rel[0]), 0.0, 1.0))
    #
    # # The axis tells you WHICH direction the rotation is in
    # axis = q_rel[1:]
    # norm = np.linalg.norm(axis)
    # if norm > 1e-6:
    #     axis = axis / norm

    return np.degrees(angle)

# --- ElbowAngleNode Class ---
class ElbowAngleNode(Node):
    def __init__(self):
        # Init
        super().__init__('angle_estimation')
        self.imu1_msg = None
        self.imu2_msg = None
        self.angle_start = None
        self.imu1_fresh = False
        self.imu2_fresh = False

        # Subscribers
        self.sub_imu1 = self.create_subscription(Imu, '/imu1/data', self.imu1_callback, 10)
        self.sub_imu2 = self.create_subscription(Imu, '/imu2/data', self.imu2_callback, 10)

        # Publisher
        self.pub_angle = self.create_publisher(Float32, '/angle', 10)
        self.get_logger().info("ElbowAngleNode started: listening to /imu1/data, /imu2/data")

    def imu1_callback(self, msg):
        self.imu1_msg = msg
        self.imu1_fresh = True
        self.estimate_and_publish()

    def imu2_callback(self, msg):
        self.imu2_msg = msg
        self.imu2_fresh = True
        self.estimate_and_publish()

    def estimate_and_publish(self):
        # Check if both messages have been received
        if not self.imu1_fresh or not self.imu2_fresh:
            return

        self.imu1_fresh = False
        self.imu2_fresh = False
        
        # Otherwise standardize quaternion from imu messages
        qUpper = quaternion_array(self.imu1_msg)   # IMU1 in upper arm
        qForearm = quaternion_array(self.imu2_msg) # IMU2 in forearm

        # Calculate angle
        angle_deg = relative_quat_method(qUpper, qForearm)

        if self.angle_start is None:
            self.angle_start = angle_deg

        # Create message to publish
        msg = Float32()
        msg.data = float(np.abs(self.angle_start - angle_deg))  # Calibrates the angle to start at 0 degrees
        # msg.data = float(abs(angle_deg))

        # Publish
        self.pub_angle.publish(msg)
        self.get_logger().info(f"Published elbow angle: {msg.data:.2f} degrees")
        
def main():
    rclpy.init()
    node = ElbowAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
