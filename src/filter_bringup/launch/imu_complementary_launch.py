# imu_complementary_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Filter for IMU1
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='imu1_filter',
            parameters=[{
                'use_mag': True,
                'publish_tf': False,
                'fixed_frame': 'odom',
                'remove_gravity_vector': True,
            }],
            remappings=[
                ('imu/data_raw', 'imu1/data_raw'),
                ('imu/mag', 'imu1/mag'),
                ('imu/data', 'imu1/data'),  # filtered output
            ]
        ),
        # Filter for IMU2
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='imu2_filter',
            parameters=[{
                'use_mag': True,
                'publish_tf': False,
                'fixed_frame': 'odom',
                'remove_gravity_vector': True,
            }],
            remappings=[
                ('imu/data_raw', 'imu2/data_raw'),
                ('imu/mag', 'imu2/mag'),
                ('imu/data', 'imu2/data'),
            ]
        ),
    ])
