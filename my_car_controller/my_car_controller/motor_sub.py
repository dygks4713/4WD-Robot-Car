import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_sub')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.move_callback, 10)
        self.get_logger().info('Subscribed to topic: /cmd_vel')

        # 핀 설정 (이전 4WD 배선 기준)
        self.PINS = {
            'FL': {'IA': 17, 'IB': 18}, 'RL': {'IA': 27, 'IB': 22},
            'FR': {'IA': 23, 'IB': 24}, 'RR': {'IA': 25, 'IB': 8}
        }
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.pwms = {}
        for motor, pins in self.PINS.items():
            GPIO.setup(pins['IA'], GPIO.OUT)
            GPIO.setup(pins['IB'], GPIO.OUT)
            # [튜닝 1] 주파수를 50Hz로 낮춰 저속 토크 확보
            self.pwms[f"{motor}_A"] = GPIO.PWM(pins['IA'], 50) 
            self.pwms[f"{motor}_B"] = GPIO.PWM(pins['IB'], 50)
            self.pwms[f"{motor}_A"].start(0)
            self.pwms[f"{motor}_B"].start(0)

        self.get_logger().info('ROS2 Motor Subscriber Node has been started')

    def setup_gpio(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.pwms = {}
        for motor, pins in self.PINS.items():
            GPIO.setup(pins['IA'], GPIO.OUT)
            GPIO.setup(pins['IB'], GPIO.OUT)
            self.pwms[f"{motor}_A"] = GPIO.PWM(pins['IA'], 100)
            self.pwms[f"{motor}_B"] = GPIO.PWM(pins['IB'], 100)
            self.pwms[f"{motor}_A"].start(0)
            self.pwms[f"{motor}_B"].start(0)

    def set_motor(self, motor, speed_a, speed_b):
        # [튜닝 2] 데드존 보상: 25 이하는 모터가 안 움직이므로 최소값을 25로 고정
        MIN_POWER = 25 
        
        if speed_a > 0: speed_a = max(MIN_POWER, min(100, speed_a))
        if speed_b > 0: speed_b = max(MIN_POWER, min(100, speed_b))
        
        self.pwms[f"{motor}_A"].ChangeDutyCycle(speed_a)
        self.pwms[f"{motor}_B"].ChangeDutyCycle(speed_b)

    def move_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # [튜닝 1] 기본 속도와 회전 가중치 설정
        base_speed = 40.0   # 전진 속도 (느리게 설정)
        angular_gain = 1.5  # 회전 시 속도를 1.5배 강화 (마찰력 극복용)
        
        # 선속도와 각속도 계산 (회전 시 더 강한 힘을 주도록 수정)
        left_speed = (linear_x - (angular_z * angular_gain)) * base_speed
        right_speed = (linear_x + (angular_z * angular_gain)) * base_speed

        # [튜닝 2] 제자리 회전 시 최소 출력 보장 (Dead-zone 보상 강화)
        # linear_x가 0이고 회전만 할 때, 바퀴가 멈춰있지 않게 최소 힘을 줍니다.
        if linear_x == 0.0 and angular_z != 0.0:
            turn_min_power = 35.0 # 회전 시 최소 PWM을 전진보다 높게 설정
            if left_speed > 0: left_speed = max(left_speed, turn_min_power)
            elif left_speed < 0: left_speed = min(left_speed, -turn_min_power)
            if right_speed > 0: right_speed = max(right_speed, turn_min_power)
            elif right_speed < 0: right_speed = min(right_speed, -turn_min_power)

        # 왼쪽 모터 제어 (FL, RL)
        if left_speed > 0:
            self.set_motor('FL', 0, abs(left_speed)); self.set_motor('RL', 0, abs(left_speed))
        elif left_speed < 0:
            self.set_motor('FL', abs(left_speed), 0); self.set_motor('RL', abs(left_speed), 0)
        else:
            self.set_motor('FL', 0, 0); self.set_motor('RL', 0, 0)

        # 오른쪽 모터 제어 (FR, RR)
        if right_speed > 0:
            self.set_motor('FR', abs(right_speed), 0); self.set_motor('RR', abs(right_speed), 0)
        elif right_speed < 0:
            self.set_motor('FR', 0, abs(right_speed)); self.set_motor('RR', 0, abs(right_speed))
        else:
            self.set_motor('FR', 0, 0); self.set_motor('RR', 0, 0)

    def stop_motors(self):
        for pwm in self.pwms.values():
            pwm.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()