import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class MotionDetector(Node):
    def __init__(self):
        super().__init__('motion_detector')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/motion_image', 10)
        self.motion_pub = self.create_publisher(Bool, '/camera/motion_detected', 10)

        self.cap = cv2.VideoCapture(8, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open /dev/video8')
            raise RuntimeError('Camera open failed')

        self.prev_gray = None
        self.min_area = 1500

        self.get_logger().info('Motion detector started on /dev/video8')
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        motion_detected = False

        if self.prev_gray is not None:
            frame_delta = cv2.absdiff(self.prev_gray, gray)
            thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)

            contours, _ = cv2.findContours(
                thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                if cv2.contourArea(contour) < self.min_area:
                    continue

                motion_detected = True
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            label = 'MOTION DETECTED' if motion_detected else 'NO MOTION'
            color = (0, 0, 255) if motion_detected else (255, 0, 0)
            cv2.putText(
                frame,
                label,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
            )

            motion_msg = Bool()
            motion_msg.data = motion_detected
            self.motion_pub.publish(motion_msg)

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(image_msg)

        self.prev_gray = gray

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
