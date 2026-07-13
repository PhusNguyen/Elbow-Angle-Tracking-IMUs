import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
import math

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

def quaternion_normalize(q):
    return q / np.linalg.norm(q)

def quaternion_rotate(q, v):
    q = quaternion_normalize(q)
    v_q = np.array([0.0, v[0], v[1], v[2]])
    return quaternion_mult(quaternion_mult(q, v_q), quaternion_conj(q))[1:]

def quat_to_euler(q):
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.degrees(math.atan2(t3, t4))

    return roll_x, pitch_y, yaw_z

# --- Angle Calculation ---
def relative_quat_EAD(qA, qB):
    # Relative rotation from upper arm to forearm
    q_rel = quaternion_mult(quaternion_conj(qA), qB)
    q_rel = quaternion_normalize(q_rel)

    # Euler Angle Decomposition
    return quat_to_euler(q_rel)

# --- ElbowAngleNode Class ---
class ElbowAngleNode(Node):
    def __init__(self):
        # __init__
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
        qUpper = quaternion_array(self.imu1_msg)
        qForearm = quaternion_array(self.imu2_msg)

        # Calculate angle
        roll, pitch, yaw = relative_quat_EAD(qUpper, qForearm)

        if self.angle_start is None:
            self.angle_start = yaw

        # Create message to publish
        msg = Float32()

        # Publish
        msg.data = np.abs(yaw - self.angle_start)
        self.pub_angle.publish(msg)
        self.get_logger().info(f"Publish Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {msg.data:.2f}")

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
