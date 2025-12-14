from mcap_ros2.reader import read_ros2_messages
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import csv

def mcap_to_csv(mcap_file, topic1, output_csv):
    """Convert IMU Rosbag mcap file to CSV"""
    
    with open(mcap_file, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        
        # # Open CSV file for writing
        # with open(output_csv, 'w', newline='') as csvfile1:
        #     fieldnames_imu = [
        #         'timestamp_sec', 'timestamp_nsec', 'frame_id',
        #         'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
        #         'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
        #         'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
        #         # 'magnetic_field_x', 'magnetic_field_y', 'magnetic_field_z',
        #     ]
        #     writer = csv.DictWriter(csvfile1, fieldnames=fieldnames_imu)
        #     writer.writeheader()
            
        #     for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=[topic1]):
        #         writer.writerow({
        #             'timestamp_sec': ros_msg.header.stamp.sec,
        #             'timestamp_nsec': ros_msg.header.stamp.nanosec,
        #             'frame_id': ros_msg.header.frame_id,
        #             'orientation_x': ros_msg.orientation.x,
        #             'orientation_y': ros_msg.orientation.y,
        #             'orientation_z': ros_msg.orientation.z,
        #             'orientation_w': ros_msg.orientation.w,
        #             'angular_velocity_x': ros_msg.angular_velocity.x,
        #             'angular_velocity_y': ros_msg.angular_velocity.y,
        #             'angular_velocity_z': ros_msg.angular_velocity.z,
        #             'linear_acceleration_x': ros_msg.linear_acceleration.x,
        #             'linear_acceleration_y': ros_msg.linear_acceleration.y,
        #             'linear_acceleration_z': ros_msg.linear_acceleration.z,
        #             # 'magnetic_field_x': ros_msg.mag_field.magnetic_field.x,
        #             # 'magnetic_field_y': ros_msg.mag_field.magnetic_field.y,
        #             # 'magnetic_field_z': ros_msg.mag_field.magnetic_field.z,
        #         })

        # Open CSV file for writing
        with open(output_csv, 'w', newline='') as csvfile1:
            fieldnames_angle = [
                'estimated_angle',
            ]
            writer = csv.DictWriter(csvfile1, fieldnames=fieldnames_angle)
            writer.writeheader()
            
            for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=[topic1]):
                writer.writerow({
                    'estimated_angle': ros_msg.data,
                })
            
    
    print(f"Data saved to {output_csv}")

# Conversion
rosbag_file = "/home/pnguyen/Workspace/Northeastern_University/RSN_EECE_5554/PROJECT/Elbow-Angle-Tracking-IMUs/rosbag_file/madgwick_rosbag_6/madgwick_rosbag_6_0.mcap"
mcap_to_csv(rosbag_file, "/angle", "angle_madgwick_5.csv")

# Specify the MCAP file to read
# for msg in read_ros2_messages(rosbag_file):
#     if msg.channel.topic == "/angle":
#         print(f"{msg.channel.topic}: {msg.ros_msg}")