import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorObjectDetector(Node):
    def __init__(self):
        super().__init__('color_object_detector')
        self.declare_parameter('color', 'red')  # can be red, purple, yellow
        self.color = self.get_parameter('color').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info(f'Look for {self.color.upper()} packages...')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define colour ranges
        if self.color == 'red':
            lower1 = np.array([0, 120, 70])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([170, 120, 70])
            upper2 = np.array([180, 255, 255])
            mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)

        elif self.color == 'yellow':
            lower = np.array([20, 100, 100])
            upper = np.array([30, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

        elif self.color == 'purple':
            lower = np.array([130, 50, 50])
            upper = np.array([160, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

        else:
            self.get_logger().warn('Unsupported color selected!')
            return

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.get_logger().info(f'{self.color.upper()} package detected!')

        cv2.imshow("Detected", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
