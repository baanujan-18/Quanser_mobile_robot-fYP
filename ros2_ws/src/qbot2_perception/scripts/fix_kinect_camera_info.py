import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoFixer(Node):
    def __init__(self):
        super().__init__('camera_info_fixer')

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
        msg.k[0] = 570.3422241210938
        msg.k[2] = 319.5
        msg.k[4] = 570.3422241210938
        msg.k[5] = 239.5
        msg.k[8] = 1.0

        msg.p[0] = 570.3422241210938
        msg.p[2] = 319.5
        msg.p[5] = 570.3422241210938
        msg.p[6] = 239.5
        msg.p[10] = 1.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CameraInfoFixer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
