import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String, Int32
import threading, time
import serial
from pynput import keyboard
from email.message import EmailMessage
import smtplib

CM_PER_STEP = int(1080/10)

class ArduinoHandlerThread(threading.Thread):
    def __init__(self, port, baudrate, email_address, email_password, target_email):
        super().__init__()
        self.arduino = serial.Serial(port, baudrate)
        self.email_address = email_address
        self.email_password = email_password
        self.target_email = target_email
        self._stop_event = threading.Event()
        self._action_event = threading.Event()  # Perform action 신호
        self.message = None  # perform_action에 전달할 메시지

    def run(self):
        # 쓰레드 실행
        # 이 작업은 루프를 통해 계속 실행되며, 중지 이벤트가 설정될 때까지 계속 실행됩니다.
        # 데이터를 읽고 조건에 따라 동작을 수행합니다.
        # Perform action 신호가 설정되면 perform_action을 호출하여 메시지를 보냅니다.
        while not self._stop_event.is_set():
            try:
                # 작업 대기
                if self._action_event.is_set():
                    # Perform action 실행
                    if self.message:
                        self.perform_action(self.message)
                        self.message = None
                    self._action_event.clear()  # perform_action 완료 후 루프 재개
                else:
                    # 데이터 감지
                    if self.arduino.in_waiting > 0:  # 수신 데이터가 있는 경우
                        line = self.arduino.readline().decode('utf-8').strip()
                        print(f"Arduino Response: {line}")

                        # 조건에 따라 동작 수행
                        if "ERROR" in line:
                            print("Error detected! Sending email...")
                            self.send_email("Conveyor Error", "Conveyor is not working properly")
                        elif "ACTION" in line:
                            print("Performing action based on Arduino message...")
                            self.start_action("1\n")  # 예: 메시지를 사용하여 perform_action 호출
                    else:
                        time.sleep(0.1)  # 데이터가 없으면 대기
            except Exception as e:
                print(f"Error in ArduinoHandlerThread: {e}")
                break

    def perform_action(self, message):
        """아두이노로 명령 전송"""
        try:
            self.arduino.write(message.encode())
            print(f"Action performed: Sent message '{message}' to Arduino.")
        except Exception as e:
            print(f"Failed to send message: {e}")

    def send_email(self, subject, body):
        """이메일 전송"""
        try:
            msg = EmailMessage()
            msg.set_content(body)
            msg['Subject'] = subject
            msg['From'] = self.email_address
            msg['To'] = self.target_email

            with smtplib.SMTP_SSL('smtp.gmail.com', 465) as smtp:
                smtp.login(self.email_address, self.email_password)
                smtp.send_message(msg)

            print("Error email sent successfully.")
        except Exception as e:
            print(f"Failed to send email: {e}")

    def start_action(self, message):
        """Perform action 실행 신호를 설정"""
        self.message = message
        self._action_event.set()

    def stop_thread(self):
        self._stop_event.set()
        self.arduino.close()


class ConveyorStateMonitorNode(Node):
    def __init__(self):
        super().__init__('conveyor_state_monitor')
        self.cmd_sub = self.create_subscription(
            Int32, '/conveyor/cmd', self.cmd_callback, 10)
        self.status_publisher = self.create_publisher(String,'/conveyor/status', 10)
        self.current_status = "정지"
        self.checking = False
        self.lock = threading.Lock()
        self.stop_listener = False
        self.thread_start()

    def thread_start(self):
        from_email ='minseung2201@gmail.com'
        email_password = 'miog vbdi eojb fdkg'
        to_email = 'na06219@g.skku.edu'
        self.monitor_thread = ArduinoHandlerThread('/dev/ttyACM1', 115200, from_email, email_password, to_email)
        self.monitor_thread.start() # 쓰레드 시작 및 모니터링 시작

    def cmd_callback(self, msg):
        cmd = msg.data
        if cmd == 1:
            self.get_logger().info('Conveyor is moving forward...')
            self.current_status = "동작중"
            message = f"{CM_PER_STEP * 10}"
            self.publish_status()
        elif cmd == -1:
            self.get_logger().info('Conveyor is moving backward...')
            self.current_status = "후진"
            message = f"{CM_PER_STEP * 80}"
            self.publish_status()
        elif cmd == 0:
            self.get_logger().info('Conveyor is stopped...')
            self.current_status = "정지"
            message = "1"
            self.publish_status()
        else:
            self.get_logger().error(f"Invalid command: {cmd}")
            return
        
        self.monitor_thread.start_action(message) # 아두이노로 메시지 전송
            

    def publish_status(self):
        """현재 상태를 발행"""
        msg = String()
        msg.data = self.current_status
        self.status_publisher.publish(msg)
        self.get_logger().info('Published: {}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorStateMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()