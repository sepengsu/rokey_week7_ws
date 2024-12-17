import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSignal, QThread
from sensor_msgs.msg import Image, CompressedImage
import sys, cv2, numpy as np
class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.conveyor_cmd_pub = self.create_publisher(Int32, '/conveyor/cmd', QoSProfile(depth=10))
        self.conveyor_status_sub = self.create_subscription(String, '/conveyor/status', self.conveyor_status_callback, QoSProfile(depth=10))
        self.world_cam_sub = self.create_subscription(CompressedImage, '/world/compressed_image', self.world_cam_callback, QoSProfile(depth=10))
        self.robot_status_pub = self.create_publisher(String, '/robot/status', QoSProfile(depth=10))
        self.conveyor_status = None 
        self.world_cam = None
        self.robot_status = 'play'

    def __del__(self):
        self.destroy_node()
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
        self.world_cam = image_np
        
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
        