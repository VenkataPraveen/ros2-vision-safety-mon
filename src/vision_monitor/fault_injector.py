import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FaultInjector(Node):
    def __init__(self):
        super().__init__('fault_injector')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/camera/faulty_image', 10)

        self.fault_mode = 'blur'   # options: none, blackout, blur, freeze, noise
        self.last_good_frame = None

        self.get_logger().info(f'Fault injector started with mode: {self.fault_mode}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        output = frame.copy()

        if self.fault_mode == 'none':
            output = frame

        elif self.fault_mode == 'blackout':
            output = np.zeros_like(frame)

        elif self.fault_mode == 'blur':
            output = cv2.GaussianBlur(frame, (31, 31), 0)

        elif self.fault_mode == 'freeze':
            if self.last_good_frame is None:
                self.last_good_frame = frame.copy()
            output = self.last_good_frame

        elif self.fault_mode == 'noise':
            noise = np.random.normal(0, 25, frame.shape).astype(np.int16)
            noisy = frame.astype(np.int16) + noise
            output = np.clip(noisy, 0, 255).astype(np.uint8)

        else:
            self.get_logger().warn(f'Unknown fault mode: {self.fault_mode}')
            output = frame

        if self.fault_mode != 'freeze':
            self.last_good_frame = frame.copy()

        cv2.putText(
            output,
            f'FAULT: {self.fault_mode.upper()}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

        out_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
