import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoFixer(Node):
    def __init__(self):
        super().__init__('kinect_camera_info_fixer')
        self.sub = self.create_subscription(
            CameraInfo,
            '/kinect/depth/camera_info',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            CameraInfo,
            '/kinect/depth/camera_info_fixed',
            10
        )

    def callback(self, msg):
        fixed = msg

        # Fix missing distortion model
        if fixed.distortion_model == '' or fixed.distortion_model == 'unknown':
            fixed.distortion_model = 'plumb_bob'

        # Kinect depth is normally 640x480
        if fixed.width == 0:
            fixed.width = 640
        if fixed.height == 0:
            fixed.height = 480

        # If distortion coefficients are missing, assume no distortion
        if len(fixed.d) == 0:
            fixed.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # If intrinsic matrix is missing, use common Kinect depth approximate values
        if fixed.k[0] == 0.0:
            fx = 525.0
            fy = 525.0
            cx = fixed.width / 2.0 - 0.5
            cy = fixed.height / 2.0 - 0.5

            fixed.k = [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0
            ]

            fixed.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]

        self.pub.publish(fixed)

def main():
    rclpy.init()
    node = CameraInfoFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
