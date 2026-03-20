import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.br = TransformBroadcaster(self)

        # Subscriber
        self.sub_imu1 = self.create_subscription(Imu, '/imu1/data', self.imu1_callback, 10)
        self.sub_imu2 = self.create_subscription(Imu, '/imu2/data', self.imu2_callback, 10)

        self.get_logger().info("IMU TF Broadcaster started")

    def imu1_callback(self, msg):
        self.publish_tf(msg, 'world', 'imu1_frame')

    def imu2_callback(self, msg):
        self.publish_tf(msg, 'world', 'imu2_frame')

    def publish_tf(self, msg, parent_frame, child_frame):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # No position offset - just orientation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Orientation from IMU
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
