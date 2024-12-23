import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSignal, QThread
from sensor_msgs.msg import Image, CompressedImage
import sys, os, cv2
import numpy as np
from cv2 import aruco
import glob
from ament_index_python.packages import get_package_share_directory

path = get_package_share_directory('project') # 패키지의 경로를 가져옴

world_para = np.load(os.path.join(path,'params','world_calib.npz')) # 세계 카메라의 파라미터

CAMERA_PARAMETERS= {key: world_para[key] for key in world_para}
CAMERA_PARAMETERS['real_size'] = (0.105,0.105)

robot_para = np.load(os.path.join(path,'params','robot_calib.npz')) # 로봇 카메라의 파라미터
ROBOT_PARAMETERS = {key: robot_para[key] for key in robot_para}
ROBOT_PARAMETERS['real_size'] = (0.105,0.105)

def get_pose(frame, parameters):
    mtx = parameters['mtx']
    dist = parameters['dist']
    k = mtx
    d = dist
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian filtering
    frame = cv2.GaussianBlur(frame, (5, 5), 0)

    # Load the predefined dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    arucoParams = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)

    # If no markers are detected, return immediately
    if ids is None or len(ids) == 0:
        return frame, None, None

    # Estimate pose of each marker and return the values rvec and tvec
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.105, k, d)

    # Compute rotation matrix for the first detected marker
    R, _ = cv2.Rodrigues(rvecs[0])

    # Draw axes, markers, and position text for each detected marker
    for i, marker_id in enumerate(ids):
        # Draw the axis on the marker
        cv2.aruco.drawAxis(frame, k, d, rvecs[i], tvecs[i], 0.1)
        # Draw a square around the detected marker
        cv2.aruco.drawDetectedMarkers(frame, corners)

        # Extract and display the position of the marker
        x = tvecs[i][0][0]
        y = tvecs[i][0][1]
        z = tvecs[i][0][2]
        cv2.putText(frame, f"x: {x:.2f} y: {y:.2f} z: {z:.2f}", 
                    (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (255, 255, 255), 2, cv2.LINE_AA)

    return frame, ids, tvecs


class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.conveyor_cmd_pub = self.create_publisher(Int32, '/conveyor/cmd', QoSProfile(depth=10))
        self.conveyor_status_sub = self.create_subscription(String, '/conveyor/status', self.conveyor_status_callback, QoSProfile(depth=10))
        self.world_cam_sub = self.create_subscription(CompressedImage, '/world/compressed_image', self.world_cam_callback, QoSProfile(depth=10))
        self.robot_status_pub = self.create_publisher(String, '/robot/status', QoSProfile(depth=10))
        self.conveyor_status = None 
        self.world_cam = None
        self.robot_image = None
        self.yolo_output = None
        self.yolo_box_list = None
        self.clss_list = None
        self.robot_status = 'play'
        self.num = 12
        self.robot_image_sub = self.create_subscription(CompressedImage, '/robot/compressed_image', self.robot_image_callback, QoSProfile(depth=10))
        self.yolo_classs_sub = self.create_subscription(String, '/robot/yolo_classes', self.yolo_classs_callback, QoSProfile(depth=10))
        self.yolo_boxes_sub = self.create_subscription(String, '/robot/yolo_boxes', self.yolo_boxes_callback, QoSProfile(depth=10))
    
    def yolo_classs_callback(self, msg):
        '''
        msg: String
        '''
        self.clss_list = eval(msg.data)
    
    def yolo_boxes_callback(self, msg):
        '''
        msg: String
        '''
        self.yolo_box_list = eval(msg.data)

    def __del__(self):
        self.destroy_node()
        rclpy.shutdown()
    def pub_to_conveyor(self, types:int):
        '''
        types: 1 -> forward, -1 -> backward, 0 -> stop
        '''
        if types not in [-1,0,1]:
            self.get_logger().error('Invalid type')
            return
        msg = Int32()
        msg.data = types
        self.conveyor_cmd_pub.publish(msg)
        self.get_logger().info('Published to /conveyor/cmd: {}'.format(msg.data))
    
    def robot_image_callback(self, msg):
        '''
        msg: CompressedImage
        '''
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image , ids, tvecs = get_pose(image_np,ROBOT_PARAMETERS)
        if ids is not None:
            self.get_logger().info('Received from /robot/compressed_image')
            self.get_logger().info(f'ids: {ids}, tvecs: {tvecs}')
        self.robot_image = image

    def conveyor_status_callback(self, msg):
        '''
        msg: Int32
        '''
        self.get_logger().info('Received from /conveyor/status: {}'.format(msg.data))
        self.conveyor_status = msg.data
    
    def world_cam_callback(self, msg):
        '''
        msg: CompressedImage
        '''
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        image, ids, tvecs = get_pose(image_np, CAMERA_PARAMETERS)
        if ids is not None:
            self.get_logger().info('Received from /world/compressed_image')
            self.get_logger().info(f'ids: {ids}, tvecs: {tvecs}')
        self.world_cam = image

    def pub_to_robot_state(self, state:str):
        '''
        msg: String
        '''
        now_state = self.robot_status
        if state == 'play': 
            if now_state == 'play':
                self.get_logger().warn('Already playing')
                return
            elif now_state == 'pause' or now_state == 'reset': # pause, reset -> play
                self.robot_status = 'play'
                self.get_logger().info('Published to /robot/status: {}'.format(state))
            elif now_state == 'stop': # not play -> play
                self.get_logger().error('Cannot play from stop state') 
                return
            return True

        elif state == 'pause':
            if now_state == 'play':
                self.robot_status = 'pause'
                self.get_logger().info('Published to /robot/status: {}'.format(state))
            if now_state == 'pause':
                self.get_logger().warn('Already paused')
                return
            if now_state == 'stop':
                self.get_logger().error('Cannot pause from stop state')
                return
            if now_state == 'reset':
                self.get_logger().error('Cannot pause from reset state')
                return
            return True
            
        elif state == 'stop':
            if now_state == 'stop':
                self.get_logger().warn('Already stopped')
                return
            else:
                self.robot_status = 'stop'
                self.get_logger().info('Published to /robot/status: {}'.format(state))
            return True
            
        elif state == 'reset':
            if now_state =='play':
                self.get_logger().error('Cannot reset from play state')
                return
            if now_state == 'pause':
                self.get_logger().error('Cannot reset from pause state')
                return
            if now_state == 'stop':
                self.robot_status = 'reset'
                self.get_logger().info('Published to /robot/status: {}'.format(state))
            if now_state == 'reset':
                self.get_logger().warn('Already reset')
                return
            
            return True # 바뀐 상태 반환 
        else:
            self.get_logger().error('Invalid state')
            return
        
    def capture_world_cam(self):
        img = self.robot_image # 로봇 카메라 이미지
        dirs = 'train/'
        if not os.path.exists(dirs):
            os.makedirs(dirs)
        num_list = glob.glob(dirs + '*.jpg') 
        num_list = [int(num.split('/')[-1].split('.')[0].split('_')[-1]) for num in num_list] # 이미 저장된 이미지들의 번호를 가져옴
        num = max(num_list) + 1 if num_list else 0 # 이미지 번호 설정
        name = f'puple_{num}.jpg' # 로봇 카메라 이미지 저장
        path = os.path.join(dirs, name)
        cv2.imwrite(path, img)
    
