import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSignal, QThread
import sys, cv2, numpy as np
from project.gui_node import GUINode
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QImage, QPixmap
class ROSNodeThread(QThread):
    """
    ROS 노드를 별도의 QThread에서 실행.
    """
    update_signal = pyqtSignal()  # GUI 업데이트를 위한 신호

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True

    def run(self):
        """
        ROS 노드 실행 루프.
        """
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.update_signal.emit()  # GUI 업데이트 신호

    def stop(self):
        """
        스레드를 종료합니다.
        """
        self.running = False
        self.wait()

class GUI(QMainWindow):
    def __init__(self,node):
        super().__init__()
        self.node = node
        self.init_ui()
        self.funtion_button()
    
    def init_ui(self):
        '''
        word_cam: 카메라 이미지를 보여주는 객체
        convey_system_log: 컨베이어 시스템 로그를 보여주는 객체
        '''
        ui_path = '/home/jaenote/rokey_week7_ws/src/project/ui/gui.ui'
        loadUi(ui_path, self)
        self.setWindowTitle('GUI')

    def funtion_button(self):
        '''
        forward_button: 전진 버튼
        backward_button: 후진 버튼
        stop_button: 정지 버튼
        play_button: 시작 버튼
        stop_reset_button: 정지 및 초기화 버튼
        pause_resume_button: 일시정지 및 재시작 버튼
        '''
        self.forward_button.clicked.connect(self.forward)
        self.backward_button.clicked.connect(self.backward)
        self.stop_button.clicked.connect(self.stop)
        self.play_button.clicked.connect(self.play)
        self.stop_reset_button.clicked.connect(self.stop_reset)
        self.pause_resume_button.clicked.connect(self.pause_resume)
    
    def forward(self):
        self.node.pub_to_conveyor(1)

    def backward(self):
        self.node.pub_to_conveyor(-1)

    def stop(self):
        self.node.pub_to_conveyor(0)

    def play(self):
        pass
    def stop_reset(self):
        pass
    def pause_resume(self):
        pass

    def update_gui(self):
        self.convey_system_log.setText('Conveyor Status: {}'.format(self.node.conveyor_status))
        if self.node.world_cam is not None:
            image_rgb = cv2.cvtColor(self.node.world_cam, cv2.COLOR_BGR2RGB)
            height, width, channel = image_rgb.shape

            # NumPy 배열을 QImage로 변환
            bytes_per_line = 3 * width
            qimage = QImage(image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # QImage를 QPixmap으로 변환 후 QLabel에 표시
            pixmap = QPixmap.fromImage(qimage)
            self.world_cam.setPixmap(pixmap)
def main(args=None):
    rclpy.init(args=args)
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QFont

    # High DPI Scaling 설정
    if hasattr(Qt, "AA_EnableHighDpiScaling"):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, "AA_UseHighDpiPixmaps"):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    node = GUINode()
    app = QApplication(sys.argv)
    gui = GUI(node)
    ros_thread = ROSNodeThread(node)
    ros_thread.update_signal.connect(gui.update_gui)
    ros_thread.start()
    gui.show()
    try:
        sys.exit(app.exec_())  # PyQt 이벤트 루프 실행
    except KeyboardInterrupt:
        ros_thread.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    from gui_node import GUINode
    main()