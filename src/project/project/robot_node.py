import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2, os 
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from std_msgs.msg import String
from std_msgs.msg import Header
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
# from cv2 import aruco

import math


j1_z_offset = 77
r1 = 130
r2 = 124
r3 = 150


th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5*math.pi - th1_offset

# author : karl.kwon (mrthinks@gmail.com)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J0 to J2
def solv2(r1, r2, r3):
  d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
  d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

  s1 = math.acos(d1 / r1)
  s2 = math.acos(d2 / r2)

  return s1, s2

def solv_robot_arm2(x, y, z, r1, r2, r3):
  z = z + r3 - j1_z_offset

  Rt = math.sqrt(x**2 + y**2 + z**2)
  Rxy = math.sqrt(x**2 + y**2)
  St = math.asin(z / Rt)
#   Sxy = math.acos(x / Rxy)
  Sxy = math.atan2(y, x)

  s1, s2 = solv2(r1, r2, Rt)

  sr1 = math.pi/2 - (s1 + St)
  sr2 = s1 + s2
  sr2_ = sr1 + sr2
  sr3 = math.pi - sr2_

  return Sxy, sr1, sr2, sr3, St, Rt

path = get_package_share_directory('project') # 패키지의 경로를 가져옴
robot_para = np.load(os.path.join(path,'params','robot_calib.npz')) # 로봇 카메라의 파라미터
ROBOT_PARAMETERS = {key: robot_para[key] for key in robot_para}
ROBOT_PARAMETERS['real_size'] = (0.105,0.105)

yolo_path = os.path.join(path,'yolo','best.pt') # yolo 모델의 경로


base_to_grip = 17.5*0.01
base_to_cam = 22*0.01
cam_to_grip = 7*0.01 # x축
base_to_box = 35*0.001
base_to_base = 5*0.01
base_to_box = 101*0.001

delta_z = base_to_cam - base_to_box
delta_x = -cam_to_grip 

def get_grip_pose(x, y,shapes = (640,480)):
    '''
    box 좌표를 grip 좌표로 변환
    '''
    origin_pos = [110,0,130]
    x_delta_image = x - shapes[0]/2
    y_delta_image = y - shapes[1]/2
    pose_x = x_delta_image +origin_pos[0] + delta_x
    pose_y = y_delta_image +origin_pos[1]
    pose_z = origin_pos[2] + delta_z
    return pose_x, pose_y, pose_z

def convert_to_string(outputs):
    '''
    YOLO를 이용하여 인식한 결과를 string으로 변환하는 함수
    outputs: YOLO를 이용하여 인식한 결과 (list 형태)
    반환: string으로 변환된 결과 (class_list, [x, y, w, h])
    '''
    if outputs is None or len(outputs) == 0:
        return "None"

    # YOLO 결과 처리
    result = outputs[0]  # 단일 이미지 결과 사용
    if not hasattr(result, "boxes"):
        return "None"

    boxes = result.boxes.xywh  # 바운딩 박스 정보
    classes = result.boxes.cls  # 클래스 정보

    # 리스트로 변환
    boxes_list = boxes.tolist()
    boxes_list = [[round(coord, 2) for coord in box] for box in boxes_list] 
    

    classes_list = [int(cls) for cls in classes.tolist()]

    # 문자열로 변환
    boxes_str = str(boxes_list)
    classes_str = str(classes_list)
    return classes_str, boxes_str


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.init_nodes()
        self.initing()
        self.init_camera()
        self.timer = self.create_timer(0.1, self.publish_image)

    def init_nodes(self):
        '''
        '''
        self.publisher_ = self.create_publisher(CompressedImage, '/robot/compressed_image', 10)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.yolo_clss_pub = self.create_publisher(String, '/robot/yolo_classes', 10)
        self.yolo_boxes_pub = self.create_publisher(String, '/robot/yolo_boxes', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
    
    def initing(self):
        self.image = None
        self.outpus = None
        self.trajectory_msg = None
        self.gripper_goal = GripperCommand.Goal()
        self.gripper_goal.command.position = 0.0
        self.gripper_goal.command.max_effort = 1.0
        self.gripper_goal.command.position = 0.0
        self.gripper_goal.command.max_effort = 1.0
        self.yolo_model = YOLO(yolo_path)
    
    def init_camera(self):
        # OpenCV 비디오 캡처 객체 생성 (카메라 0번 장치 사용)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

        print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
     
    def publish_image(self):
        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        if ret:
            # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # 압축된 이미지를 CompressedImage 메시지로 변환
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.format = "jpeg"
            msg.data = compressed_image.tobytes()
        
            # CompressedImage 퍼블리시
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing compressed image...')
            self.image = frame # 이미지 생성 
            result = self.detection(frame) # yolo를 이용하여 물체를 인식
            if result is not None:
                self.outputs = result
                self.get_logger().info('Publishing yolo output...')
                classes_str, boxes_str = convert_to_string(result)
                self.yolo_clss_pub.publish(String(data=classes_str))
                self.yolo_boxes_pub.publish(String(data=boxes_str))
            # self.control()
        
    def detection(self, image):
        '''
        yolo를 이용하여 물체를 인식하는 함수
        '''
        result = self.yolo_model(image)
        return result
    
    def control(self,z):
        '''
        로봇을 제어하기 위한 함수
        1. 로봇과 front_1의 위치를 인식
        '''
        z = round(z,2)
        if z == 0.20:
            self.stop()
            self.pose_to_detect()
        elif z > 0.20:
            self.go_front()
        elif z< 0.2:
            self.go_back()

    
    def stop(self):
        move = Twist()
        move.linear.x = 0
        self.get_logger().info('Stop')
        self.cmd_vel.publish(move)
    def go_front(self):
        move = Twist()
        move.linear.x = 0.1
        self.get_logger().info('Go Front')
        self.cmd_vel.publish(move)
    def go_back(self):
        move = Twist()
        move.linear.x = -0.1
        self.get_logger().info('Go Back')
        self.cmd_vel.publish(move)
    
    def pose_to_detect(self):
        '''
        로봇의 위치를 인식하기 위한 함수
        '''

        Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(110, 0, 130, r1, r2, r3)
        self.trajectory_msg = JointTrajectory()

        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()

        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        point.velocities = [0.0] * 4
        point.accelerations = [0.0] * 4
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500

        self.trajectory_msg.points = [point]

        self.joint_pub.publish(self.trajectory_msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    rclpy.shutdown()
