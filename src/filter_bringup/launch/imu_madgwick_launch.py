# imu_complementary_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Filter for IMU1
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_imu1',
            parameters=[{
                'use_mag': False,
                'publish_tf': True,
                'fixed_frame': 'world',
                'remove_gravity_vector': True,
            }],
            remappings=[
                ('imu/data_raw', 'imu1/data_raw'),
                ('imu/mag', 'imu1/mag'),
                ('imu/data', 'imu1/data'), # Output topic
            ]
        ),
        # Filter for IMU2
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu2_filter',
            parameters=[{
                'use_mag': False,
                'publish_tf': True,
                'fixed_frame': 'world',
                'remove_gravity_vector': True,
            }],
            remappings=[
                ('imu/data_raw', 'imu2/data_raw'),
                ('imu/mag', 'imu2/mag'),
                ('imu/data', 'imu2/data'),
            ]
        ),
    ])
