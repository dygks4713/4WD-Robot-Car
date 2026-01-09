import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image, 'video_frames', self.listener_callback, 10)
        self.br = CvBridge()
        # 노드 시작 메시지 출력
        self.get_logger().info('카메라 구독자 노드가 시작되었습니다. 영상을 수신합니다...')
        self.get_logger().info('exit: q')

    def listener_callback(self, data):
        # 받은 ROS2 메시지를 다시 OpenCV 이미지로 변환
        current_frame = self.br.imgmsg_to_cv2(data)
        
        # 화면에 영상 표시
        cv2.imshow("Robot Camera View", current_frame)
        
        # 'q' 키를 누르면 종료 처리
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('사용자에 의해 종료 명령(q)이 입력되었습니다.')
            # 노드를 안전하게 종료하기 위해 예외 발생 (main에서 캐치)
            raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        # 'q' 클릭 또는 Ctrl+C 시 메시지 출력
        print('\n프로그램을 종료합니다.')
    finally:
        # 창 닫기 및 노드 파괴
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()