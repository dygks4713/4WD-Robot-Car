import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios

# 2. 키보드 입력을 엔터 없이 읽어오는 함수 (기존 코드 활용)
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        # /cmd_vel 토픽으로 Twist 메시지를 발행합니다.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop Node가 시작되었습니다.')
        self.get_logger().info('W: 전진, S: 후진, A: 좌회전, D: 우회전, Space: 정지, Q: 종료')

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = getch()
                
                # 기존 키 매핑을 Twist 메시지로 변환 (속도 1.0 기준)
                if key == 'w':
                    twist.linear.x = 1.0; twist.angular.z = 0.0
                elif key == 's':
                    twist.linear.x = -1.0; twist.angular.z = 0.0
                elif key == 'a':
                    twist.linear.x = 0.0; twist.angular.z = 1.0
                elif key == 'd':
                    twist.linear.x = 0.0; twist.angular.z = -1.0
                elif key == ' ':
                    twist.linear.x = 0.0; twist.angular.z = 0.0
                elif key == 'q':
                    self.get_logger().info('종료합니다.')
                    break
                else:
                    continue
                
                self.publisher_.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'에러 발생: {e}')
        finally:
            # 종료 시 정지 신호 전송
            twist.linear.x = 0.0; twist.angular.z = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()