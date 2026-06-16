from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/kinect/depth/image_raw'),
                ('depth_camera_info', '/kinect/depth/camera_info_fixed'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'scan_height': 10,
                'range_min': 0.45,
                'range_max': 4.0,
                'output_frame': 'kinect_depth_frame',
            }]
        )
    ])
