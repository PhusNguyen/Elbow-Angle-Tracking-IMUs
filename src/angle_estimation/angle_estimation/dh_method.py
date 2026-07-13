import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np

def quaternion_array(q):
    q_array = np.array([
        q.orientation.w,
        q.orientation.x,
        q.orientation.y,
        q.orientation.z
    ], dtype=float)
    return q_array

def quaternion_to_rotation_matrix(q):
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x*x + y*y)]
    ])

def dh_transform(theta, alpha, a, d):
    """Single DH transformation matrix from parameters."""
    theta = (theta)
    ct, st = np.cos(np.radians(theta)), np.sin(np.radians(theta))
    ca, sa = np.cos(np.radians(alpha)), np.sin(np.radians(alpha))
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def dh_method(qA, qB):
    """
    Extract elbow angles using DH parameter model.

    qA: quaternion from IMU1 (upper arm) [w, x, y, z]
    qB: quaternion from IMU2 (forearm) [w, x, y, z]

    Returns: theta (flexion), alpha (carrying angle) in degrees
    """
    # Convert quaternions to rotation matrices
    R_A = quaternion_to_rotation_matrix(qA)
    R_B = quaternion_to_rotation_matrix(qB)

    # Relative rotation: upper arm frame to forearm frame
    R_rel = R_A.T @ R_B

    # Extract angles from DH rotation matrix structure:
    # R = [cθ   -cα·sθ   sα·sθ]
    #     [sθ    cα·cθ  -sα·cθ]
    #     [0     sα       cα   ]

    # From R[2][1] = sin(α) and R[2][2] = cos(α)
    alpha = np.arctan2(R_rel[2, 1], R_rel[2, 2])

    # From R[1][0] = sin(θ) and R[0][0] = cos(θ)
    theta = np.arctan2(R_rel[1, 0], R_rel[0, 0])

    return np.degrees(theta), np.degrees(alpha)

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

        # Calculate angles
        theta, alpha = dh_method(qUpper, qForearm)
        if self.angle_start is None:
            self.angle_start = theta

        # Create message to publish
        msg = Float32()
        msg.data = np.abs(theta - self.angle_start)
        self.pub_angle.publish(msg)
        self.get_logger().info(f"Flex: {msg.data:2f}, Sub: {alpha:2f}")

        # Verify by reconstructing
        # T = dh_transform(msg.data, 0, 15, 0) @ dh_transform(0, alpha, 20, 0)
        # R_reconstructed = T[:3, :3]
        # print(R_reconstructed)

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
