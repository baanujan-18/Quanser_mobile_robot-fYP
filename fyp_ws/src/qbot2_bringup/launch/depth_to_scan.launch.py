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
                ('depth_camera_info', '/kinect/depth/camera_info'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'scan_height': 10,
                'range_min': 0.50,
                'range_max': 2.50,
                'output_frame': 'kinect_depth_frame',
            }]
        )
    ])
