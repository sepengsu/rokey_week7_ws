import os
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtGui import QImage, QPixmap
from project.gui_node import GUINode
import cv2
import sys
from ament_index_python import get_package_share_directory
dirname = get_package_share_directory('project')
ui_path = os.path.join(dirname, 'ui', 'gui.ui')


class ROSNodeThread(QThread):
    """
    ROS 노드를 별도의 QThread에서 실행.
    """
    update_signal = pyqtSignal(dict)  # 데이터를 전달하는 신호

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True

    def run(self):
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # 노드에서 필요한 데이터를 추출하여 전달
            data = {
                "world_cam": self.node.world_cam,
                "robot_image": self.node.robot_image,
                "yolo_boxes": self.node.yolo_box_list,
                "yolo_classes": self.node.clss_list,
                "conveyor_status": self.node.conveyor_status,
            }
            self.update_signal.emit(data)

    def stop(self):
        self.running = False
        self.wait()


class GUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
        self.init_buttons()

    def init_ui(self):
        loadUi(ui_path, self)
        self.setWindowTitle("GUI")
        self.initing()

    def initing(self):
        self.is_play = False
        self.is_pause = False
        self.is_stop = False

    def init_buttons(self):
        self.forward_button.clicked.connect(self.forward)
        self.backward_button.clicked.connect(self.backward)
        self.stop_button.clicked.connect(self.stop)
        self.play_button.clicked.connect(self.play)
        self.stop_reset_button.clicked.connect(self.stop_reset)
        self.pause_resume_button.clicked.connect(self.pause)
        self.cap_button.clicked.connect(self.cap_start)

    def cap_start(self):
        self.node.capture_world_cam()

    def forward(self):
        self.node.pub_to_conveyor(1)

    def backward(self):
        self.node.pub_to_conveyor(-1)

    def stop(self):
        self.node.pub_to_conveyor(0)

    def play(self):
        text = self.play_button.text()
        is_change = self.node.pub_to_robot_state("play")
        if is_change:
            self.play_button.setText("재시작" if text == "시작" else "시작")

    def stop_reset(self):
        text = self.stop_reset_button.text()
        output = "stop" if text == "정지" else "reset"
        is_change = self.node.pub_to_robot_state(output)
        if is_change:
            self.stop_reset_button.setText("초기화" if text == "정지" else "정지")
            if self.play_button.text() == "재시작":
                self.play_button.setText("시작")

    def pause(self):
        is_pause = self.node.pub_to_robot_state("pause")
        if is_pause and self.play_button.text() == "시작":
            self.play_button.setText("재시작")

    def update_gui(self):
        '''
        wolrd_cam: 월드 카메라 이미지 업데이트
        크기: world_cam_label 크기에 맞게 조정
        '''
        self.convey_system_log.setText('Conveyor Status: {}'.format(self.node.conveyor_status))
        if self.node.world_cam is not None:
            image_rgb = cv2.cvtColor(self.node.world_cam, cv2.COLOR_BGR2RGB)
            height, width, channel = image_rgb.shape

            # NumPy 배열을 QImage로 변환
            bytes_per_line = 3 * width
            qimage = QImage(image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # 이미지를 QLabel에 표시하기 위해 resize
            qimage = qimage.scaled(self.world_cam.width(), self.world_cam.height())
            # QImage를 QPixmap으로 변환 후 QLabel에 표시
            pixmap = QPixmap.fromImage(qimage)
            self.world_cam.setPixmap(pixmap)
            
        if self.node.robot_image is not None:
            image_rgb = cv2.cvtColor(self.node.robot_image, cv2.COLOR_BGR2RGB)
            height, width, channel = image_rgb.shape
            if self.node.yolo_box_list is not None and self.node.clss_list is not None:
                for box,cls in zip(self.node.yolo_box_list,self.node.clss_list):
                    x,y,w,h = box
                    x1 = int(x - w/2)
                    y1 = int(y - h/2)
                    x2 = int(x + w/2)
                    y2 = int(y + h/2)
                    
                    cv2.rectangle(image_rgb,(x1,y1),(x2,y2),(0,255,0),2)
                    center_x = int(x)
                    center_y = int(y)
                    cv2.circle(image_rgb, (center_x, center_y), 5, (0, 0, 255), -1)
                    cls = str(cls)
                    cv2.putText(image_rgb,cls,(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX,0.9,(36,255,12),2)

            # NumPy 배열을 QImage로 변환
            bytes_per_line = 3 * width
            qimage = QImage(image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # 이미지를 QLabel에 표시하기 위해 resize
            qimage = qimage.scaled(self.robot_cam.width(), self.robot_cam.height())
            # QImage를 QPixmap으로 변환 후 QLabel에 표시
            pixmap = QPixmap.fromImage(qimage)
            self.robot_cam.setPixmap(pixmap)
            center_x, center_y = width // 2, height // 2

            # Draw vertical line
            cv2.line(image_rgb, (center_x, 0), (center_x, height), (255, 0, 0), 2)
            # Draw horizontal line
            cv2.line(image_rgb, (0, center_y), (width, center_y), (255, 0, 0), 2)
            # Draw point at (0, 0)
            cv2.circle(image_rgb, (0, 0), 5, (0, 0, 255), -1)

            # NumPy 배열을 QImage로 변환
            bytes_per_line = 3 * width
            qimage = QImage(image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # 이미지를 QLabel에 표시하기 위해 resize
            qimage = qimage.scaled(self.robot_cam.width(), self.robot_cam.height())
            # QImage를 QPixmap으로 변환 후 QLabel에 표시
            pixmap = QPixmap.fromImage(qimage)
            self.robot_cam.setPixmap(pixmap)

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

if __name__ == "__main__":
    from project.gui_node import GUINode
    main()
