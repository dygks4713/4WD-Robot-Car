import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        
        # 1. 타이머 주기를 0.05초로 변경 (정확히 20 FPS)
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)
        
        # V4L2 백엔드로 카메라 열기 및 해상도 최적화
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.br = CvBridge()
        self.get_logger().info('카메라 노드가 시작되었습니다.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # OpenCV 이미지를 ROS2 메시지로 변환하여 발행
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()