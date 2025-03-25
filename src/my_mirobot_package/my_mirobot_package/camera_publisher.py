import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import Image # type: ignore
from cv_bridge import CvBridge # type: ignore
import cv2 # type: ignore

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.declare_parameter('camera_index', 0)  # Declare a parameter for camera index
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.cap = cv2.VideoCapture(camera_index)  # Use the parameter value
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # Publish at 20 Hz
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_message)
        else:
            self.get_logger().error('Failed to capture image')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 