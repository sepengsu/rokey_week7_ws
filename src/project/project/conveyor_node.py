import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String, Int32
import threading, time
from pynput import keyboard
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

    def cmd_callback(self, msg):
        '''
        msg: Int32 
        -1: 후진, 0: 정지, 1: 전진
        전진 상태면 일정 시간 후 정상 동작 완료로 status pub
        만약 일정 시간동안 안에 keyboard 입력이 있으면 오류 상태로 status pub
        '''
        cmd = msg.data
        if cmd == 1:
            self.get_logger().info('Conveyor is moving forward...')
            self.current_status = "동작중"
            self.publish_status()
        elif cmd == -1:
            self.get_logger().info('Conveyor is moving backward...')
            self.current_status = "후진"
            self.publish_status()
        elif cmd == 0:
            self.get_logger().info('Conveyor is stopped...')
            self.current_status = "정지"
            self.publish_status()
        else:
            self.get_logger().error('Invalid command: {}'.format(cmd))
            return 
    
        self.checking = True
        self.check_state()

    def publish_status(self):
        """현재 상태를 발행"""
        msg = String()
        msg.data = self.current_status
        self.status_publisher.publish(msg)
        self.get_logger().info('Published: {}'.format(msg.data))

    def check_state(self):
        """일정 시간 동안 키보드 입력이 없으면 정상 동작 완료로 설정
        일정 시간 동안 키보드 입력이 있으면 오류로 설정"""
        def on_press(key):
            """키 입력 이벤트 핸들러"""
            try:
                if key.char == 't':  # 't' 키 감지
                    with self.lock:
                        self.current_status = "오류"
                        self.publish_status()
                        self.get_logger().warn("Conveyor is not working properly")
                        self.stop_listener = True
                        self.checking = False
            except AttributeError:
                pass  # 특수 키 예외 처리

        # 초기 상태 설정
        if not self.checking:
            return
        self.get_logger().info("Conveyor is going to start")

        self.stop_listener = False
        init_time = time.time()

        # 키보드 리스너를 별도 스레드에서 실행
        listener = keyboard.Listener(on_press=on_press)
        listener.start()

        self.get_logger().info("Conveyor is checking...")
        while True:
            # 5초 경과 시 정상 동작 완료
            if time.time() - init_time > 5:
                with self.lock:
                    self.current_status = "정상 동작 완료"
                    self.publish_status()
                    self.get_logger().info("Conveyor is working properly")
                    self.stop_listener = True
                    self.checking = False
                break
            if self.stop_listener:  # 키 입력 감지 시 종료
                break
            time.sleep(0.1)

        listener.stop() 

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