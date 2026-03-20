from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    angle_estimation_node = Node(
        package='angle_estimation',
        executable='relative_quat_method',
        name='relative_quat_method',
        output='screen',
    )

    return LaunchDescription([
        angle_estimation_node
    ])
