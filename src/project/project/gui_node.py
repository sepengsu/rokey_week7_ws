import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSignal, QThread
import sys
class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.conveyor_cmd_pub = self.create_publisher(Int32, '/conveyor/cmd', QoSProfile(depth=10))
        self.conveyor_status_sub = self.create_subscription(String, '/conveyor/status', self.conveyor_status_callback, QoSProfile(depth=10))
        self.world_cam_sub = self.create_subscription(String, '/world/r', self.world_cam_callback, QoSProfile(depth=10))
        self.conveyor_status = None 
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