import rclpy, cv2, numpy as np, os
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CompressedImage # CompressedImage 메시지 타입 임포트 
from geometry_msgs.msg import Point # Point 메시지 타입 임포트 
from std_msgs.msg import String # String 메시지 타입 임포트
import math, time
from threading import Thread
from PyQt5.QtCore import pyqtSignal
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Int32
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
path = get_package_share_directory('project') # 패키지의 경로를 가져옴

world_para = np.load(os.path.join(path,'params','world_calib.npz')) # 세계 카메라의 파라미터

CAMERA_PARAMETERS= {key: world_para[key] for key in world_para}
CAMERA_PARAMETERS['real_size'] = (0.105,0.105)

robot_para = np.load(os.path.join(path,'params','robot_calib.npz')) # 로봇 카메라의 파라미터
ROBOT_PARAMETERS = {key: robot_para[key] for key in robot_para}
ROBOT_PARAMETERS['real_size'] = (0.105,0.105)
ROBOT_PARAMETERS['box_size'] = (0.10,0.10) # 박스의 크기

POSE_DICT= {
    'Yolo_Box_Detect': [110,0,130],
    'Convey_Detect':[0,-240,0], # 컨베이어 place 위치
    'puple_box_detect':[110,-240,130], # 보라색 박스 위치
    'deliver': [80,0,80], # 배달시 위치
    'place' : [0,240,0], # 박스를 놓을 위치
    'look': [10,0,130] # 박스를 보기위한 위치
}
class PointToPose():
    def __init__(self):
        '''
        point(x,y,z)를 받아서 로봇의 팔의 각도로 변환 
        '''
        self.j1_z_offset = 77
        self.r1 = 130
        self.r2 = 124
        self.r3 = 150

    def __call__(self, x, y, z):
        pose_msg = self.pose(x, y, z)
        return pose_msg

    def solv2(self):
        r1 = self.r1
        r2 = self.r2
        r3 = self.r3
        d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
        d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

        s1 = math.acos(d1 / r1)
        s2 = math.acos(d2 / r2)

        return s1, s2
    
    def solv_robot_arm2(self, x, y, z):
        z = z + self.r3 - self.j1_z_offset

        Rt = math.sqrt(x**2 + y**2 + z**2)
        Rxy = math.sqrt(x**2 + y**2)
        St = math.asin(z / Rt)
        Sxy = math.atan2(y, x)

        s1, s2 = self.solv2()

        sr1 = math.pi/2 - (s1 + St)
        sr2 = s1 + s2
        sr2_ = sr1 + sr2
        sr3 = math.pi - sr2_

        return Sxy, sr1, sr2, sr3, St, Rt
    
    def pose(self, x, y, z):
        '''
        로봇의 x,y,z 좌표를 받아서 로봇의 팔을 움직임
        '''
        Sxy, sr1, sr2, sr3, St, Rt = self.solv_robot_arm2(x, y, z)
        self.trajectory_msg = JointTrajectory()

        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()

        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1, sr2, sr3]
        point.velocities = [0.0] * 4
        point.accelerations = [0.0] * 4
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500

        self.trajectory_msg.points = [point]
        return self.trajectory_msg # 메시지 반환
    
class YoloPose():
    def __init__(self,K, real_size):
        '''
        yolo에서 받은 결과를 받아서 처리하는 클래스
        '''
        self.K = K
        self.real_size = real_size

    def __call__(self,box: list,origin: list):
        '''
        yolo에서 받은 결과를 받고 원래 카메라의 위치를 받이서 3d 위치를 계산
        box: [x,y,w,h]
        origin: [x,y,z] # 카메라의 위치
        '''
        x, y, w, h = box
        Z = self.calculate_z_from_cam(self.real_size, [w,h], self.K)
        X, Y, Z = self.calculate_3d_position_from_cam(x, y, Z, self.K)
        X_pos, Y_pos, Z_pos = self.calculate_3d_pos_for_box([X,Y,Z],origin)
        return X_pos, Y_pos, Z_pos
    
    def calculate_z_from_cam(self, image_size):
        """
        카메라 중심을 기준으로 Z 위치 계산 (카메라 좌표계)
        W_real: 물체의 실제 크기 (미터)
        W_image: 물체의 이미지 상 크기 (픽셀) (w,h)로 받음 
        K: 카메라 내부 파라미터 행렬
        output: Z 위치 (미터)
        """
        f_x, f_y = self.K[0, 0], self.K[1, 1]
        # Z 계산
        Z_x = (self.real_size[0] * f_x) / image_size[0]
        Z_y = (self.real_size[1] * f_y) / image_size[1]
        Z = (Z_x + Z_y) / 2 # 평균값 사용
        return Z
    
    def calculate_3d_position_from_cam(self, u, v, Z):
        """
        3D 위치 추정  카메라 중심을 기준으로 3D 위치 계산 (카메라 좌표계)
        u, v: 이미지 상의 객체 중심 좌표 (픽셀)
        Z: 카메라 좌표계에서의 Z 위치 (미터)
        K: 카메라 내부 파라미터 행렬
        """
        c_x, c_y = self.K[0, 2], self.K[1, 2]
        f_x, f_y = self.K[0, 0], self.K[1, 1]

        # 3D 위치 계산
        X = (u - c_x) * Z / f_x
        Y = (v - c_y) * Z / f_y

        return X, Y, Z
    
    def calculate_3d_pos_for_box(self, box_pose,origin):

        X,Y,Z = box_pose

        X = X + origin[0]
        Y = Y + origin[1]
        Z = Z + origin[2]

        return X,Y,Z

ID_DICT = {
    1: 'left_1',
    2: 'left_2',
    3: 'left_3',
    4: 'front_1',
    5: 'robot'
}

def get_pose(frame, parameters):
    mtx = parameters['mtx']
    dist = parameters['dist']
    k = mtx
    d = dist
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    arucoParams = cv2.aruco.DetectorParameters_create()

    corners, ids, _ = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.105, k, d)
    R, _ = cv2.Rodrigues(rvecs[0])

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.105, k, d)
    if ids is None:
        return frame, None, None
    for i in range(len(ids)):
        # 축 및 마커 그리기
        cv2.aruco.drawAxis(frame, parameters['mtx'], parameters['dist'], rvecs[i], tvecs[i], 0.105)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        x = tvecs[i][0][0]
        y = tvecs[i][0][1]
        z = tvecs[i][0][2]
        cv2.putText(frame, f"x: {x:.2f} y: {y:.2f} z: {z:.2f}", (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    return frame, ids, tvecs

'''
base_to_grip = 17.5*0.01
base_to_cam = 22*0.01
cam_to_grip = 7*0.01 # x축
base_to_box = 35*0.001
base_to_base = 5*0.01
base_to_box = 101*0.001

delta_z = base_to_cam - base_to_box
delta_x = -cam_to_grip 
'''


class ProcessThread(Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True  # 스레드 실행 여부 플래그

    def run(self):
        """스레드에서 실행할 메인 로직"""
        self.node.get_logger().info('Process thread started')
        while self.running:
            try:
                self.node.process()  # Node의 process 함수 호출
                break  # process 함수가 완료되면 루프 종료
            except Exception as e:
                self.node.get_logger().error(f"Error in process thread: {e}")
                break

    def stop(self):
        """스레드 종료"""
        self.running = False


class ControllerTower(Node):
    
    def __init__(self):
        super().__init__('controller_tower')

        self.images_pubs()
        self.points_pubs()
        self.controller_pubs()
        self.yolo_init()
        # self.command_pubs()
        # self.status_subs()
        self.thread_init()

    def yolo_init(self):
        self.yolo_pose = YoloPose(ROBOT_PARAMETERS['mtx'], ROBOT_PARAMETERS['box_size']) # yolo pose 객체 생성
        self.boxes_sub = self.create_subscription(String, '/robot/yolo_boxes', self.boxes_callback, QoSProfile(depth=5))
        self.clss_sub = self.create_subscription(String, '/robot/yolo_clss', self.clss_callback, QoSProfile(depth=5))
        self.boxes_list = None
        self.clss_list = None
    
    def controller_pubs(self):
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.case = 1

    def images_pubs(self):
        self.world_cam_sub = self.create_subscription(CompressedImage, '/world/compressed_image', self.world_cam_callback, QoSProfile(depth=10))
        self.robot_image_sub = self.create_subscription(CompressedImage, '/robot/compressed_image', self.robot_image_callback, QoSProfile(depth=10))
        self.robot_image = None
        self.world_cam = None

    def points_pubs(self):
        self.robot_position_pub = self.create_publisher(Point, '/robot/position', QoSProfile(depth=10))
        self.robot_position = None
        self.left_1_position = None
        self.left_2_position = None
        self.left_3_position = None
        self.front_1_position = None
        self.robot_front_1_position = None

    def conveyor_pubs(self):
        self.conveyor_cmd_pubs = self.create_publisher(String, '/conveyor/cmd', QoSProfile(depth=10))
    def status_subs(self):
        self.robot_status_subs = self.create_subscription(String, '/robot/status', self.robot_status_callback, QoSProfile(depth=10))
        self.conveyor_status_subs = self.create_subscription(String, '/conveyor/status', self.conveyor_status_callback, QoSProfile(depth=10))
    
    def status_pubs(self):
        self.robot_status_pubs = self.create_publisher(String, '/robot/status', QoSProfile(depth=10))
        self.conveyor_status_pubs = self.create_publisher(String, '/conveyor/status', QoSProfile(depth=10))
        self.gui_status_pubs = self.create_publisher(String, '/gui/status', QoSProfile(depth=10))
    
    def command_subs(self):
        self.gui_command_sub = self.create_subscription(String, '/gui/command', self.gui_command_callback, QoSProfile(depth=10))
        self.box_dict = None
        self.goal_id = None

    def thread_init(self):
        self.process_thread = ProcessThread(self)
        self.process_thread.start()
    def gui_command_callback(self, msg: String):
        '''
        GUI로부터 명령을 받아서 처리하는 콜백 함수
        형식: 
        {red:n, blue:m}, {id}
        '''
        if msg.data == 'None':
            self.box_dict = None
            self.goal_id = None
            return
        command = msg.data.split(',')
        box_command = command[0]
        box_dict = {box.split(':')[0]: int(box.split(':')[1]) for box in box_command.split()} # 박스 명령을 딕셔너리로 변환
        self.box_dict = box_dict
        self.goal_id = int(command[1])

    def boxes_callback(self, msg: String):
        if msg.data == 'None':
            self.boxes_list = None
            return
        self.boxes_list = msg.data.split(',')
    
        self.boxes = [list(map(int, box.split())) for box in self.boxes_list]
    
    def clss_callback(self, msg: String):
        self.clss_list = msg.data.split(',')
        self.clss = [clss for clss in self.clss_list]
        clss_dict = {0: 'blue', 1: 'purple', 2: 'red'}
        self.clss = [clss_dict[int(clss)] for clss in self.clss_list] # 클래스 번호를 클래스 이름으로 변환

    def world_cam_callback(self, msg: CompressedImage):
        '''
        세계 카메라 콜백 함수
        1. CompressedImage 메시지를 받아서 numpy 이미지로 변환
        2. 카메라 파라미터를 사용하여 이미지에 마커 위치 표시
        3. 표시된 이미지를 
        '''
        np_arr = np.frombuffer(msg.data, np.uint8) # CompressedImage 메시지를 numpy 배열로 변환
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # 이미지 디코딩
        image, ids, tvecs = get_pose(image, CAMERA_PARAMETERS)
        self.world_cam = image
        if tvecs is None or ids is None: # 마커가 없으면 함수 종료
            self.get_logger().info('No markers found')
            return 
        for mark_id , position in zip(ids, tvecs):
            types = ID_DICT[mark_id[0]] # 마커의 id를 통해 마커의 종류를 알아냄
            if types == 'robot':
                self.robot_position = position[0]
                # self.robot_position_pub.publish(Point(x=position[0],y=position[1],z=position[2]))
            elif types == 'left_1':
                self.left_1_position = position[0]
            elif types == 'left_2':
                self.left_2_position = position[0]
            elif types == 'left_3':
                self.left_3_position = position[0]
            elif types == 'front_1':
                self.front_1_position = position[0]
            else:
                self.get_logger().info('No markers found')


    def robot_image_callback(self, msg: CompressedImage):
        '''
        로봇 카메라 콜백 함수  --> 
        1. CompressedImage 메시지를 받아서 numpy 이미지로 변환
        2. 카메라 파라미터를 사용하여 이미지에 마커 위치 표시
        3. 표시된 이미지를 
        '''
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image, ids, tvecs = get_pose(image, ROBOT_PARAMETERS)
        self.robot_image = image
        if tvecs is None or ids is None:
            self.get_logger().info('No markers found')
            return
        for mark_id , position in zip(ids, tvecs):
            types = ID_DICT[mark_id[0]]
            if types == 'front_1':
                self.robot_front_1_position = position[0]
        print('Robot Front 1 Position: ',self.robot_front_1_position)

    def stop(self):
        move = Twist()
        move.linear.x = 0
        self.cmd_vel_pub.publish(move)
        self.get_logger().info('Stop')
    def go_front(self):
        move = Twist()
        move.linear.x = 0.1
        self.cmd_vel_pub.publish(move)
        self.get_logger().info('Go Front')
    def go_back(self):
        move = Twist()
        move.linear.x = -0.1
        self.cmd_vel_pub.publish(move)
        self.get_logger().info('Go Back')
    def pose(self, pose):
        self.joint_pub.publish(pose)
        self.get_logger().info('Pose Done')
    def send_gripper_goal(self, position):
            goal = GripperCommand.Goal()
            goal.command.position = position
            goal.command.max_effort = -1.0
            if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("Gripper action server not available!")
                return
            self.gripper_action_client.send_goal_async(goal)
    def grip(self):
        self.send_gripper_goal(-0.015)
        self.get_logger().info('Grip Done')
    def ungrip(self):
        self.send_gripper_goal(0.01)
        self.get_logger().info('Ungrip Done')
        

    def process(self):
        '''
        1. process1: 박스 위치로 이동하는 함수 
        2. process2: 박스를 집고 배달하는 함수
        3. process3: 보라색 박스를 찾아 움직이는 함수
        4. process4: 보라색 박스를 집어 올리는 함수 
        5. process5: 보라색 박스를 배달하기 위한 위치로 이동하는 함수
        6. process6: 보라색 박스를 배달하여 내려놓는 함수
        6. process7: 다시 원래 위치로 돌아가는 함수
        '''
        self.count = 0
        # self.process1()
        # self.get_logger().info('process1 done')
        self.process2()
        self.get_logger().info('process2 done')
        self.process3()
        self.get_logger().info('process3 done')
        self.process4()
        self.get_logger().info('process4 done')
        self.process5()
        self.get_logger().info('process5 done')
        self.process6()
        self.get_logger().info('process6 done')
    
    def process1(self):
        '''
        1. 로봇의 위치를 인식 후 move
        2. 해당 위칭 알면 정지 후 pose
        '''
        done = False
        while not done:
            if self.robot_front_1_position is None:
                if self.count < 5:
                    self.count += 1
                    time.sleep(1)
                    return
                self.get_logger().info('No markers found')
                rclpy.shutdown()
            z = round(self.robot_front_1_position[2],2)
            if z == 0.20:
                self.stop()
                pose = PointToPose()
                point = POSE_DICT['Yolo_Box_Detect']
                pose_msg = pose.pose(point[0],point[1],point[2])
                self.pose(pose_msg)
                done = True
            elif z > 0.20:
                self.go_front()
            elif z< 0.2:
                self.go_back()

    def process2(self):
        '''
        1. box를 인식 
        2. 박스 개수와 cmd 받은 박스 종류와 개수가 맞으면 pass, 아니면 error 메시지 출력
        3. 박스의 위치를 계산
        for box in boxes:
            1. 박스의 위치를 계산
            2. 로봇의 pose 각도를 계산
            3. 로봇의 pose로 이동 
            4. 로봇의 grip
            5. z축 방향으로 up (10cm)
            6. 배달 위치로 이동
            7. 로봇의 ungrip
            8. 원 위치로 이동
            9. conveyor로 80cm 이동 명령 (optional)
        '''
        boxes = self.boxes
        if len(boxes) != self.box_num:
            self.get_logger().info('Box num is not correct')
            return
        for box in boxes:
            x,y,z = self.yolo_pose(box,self.robot_front_1_position)
            pose = PointToPose()
            pose_msg = pose(x,y,z) # 로봇의 팔을 움직이기 위한 메시지
            self.pose(pose_msg)
            time.sleep(0.3)
            self.get_logger().info('Pose Done')
            
            self.grip()
            time.sleep(0.1)
            self.get_logger().info('Grip Done')

            pose_msg = pose(x,y,z+0.1) # 로봇의 팔을 움직이기 위한 메시지
            self.pose(pose_msg)
            time.sleep(0.3)

            point = POSE_DICT['deliver'] # 배달 위치
            pose_msg = pose(point[0],point[1],point[2])
            self.pose(pose_msg)
            time.sleep(0.3)

            self.ungrip() # 박스 놓기
            time.sleep(0.1)

            point = POSE_DICT['Yolo_Box_Detect'] # 원 위치
            pose_msg = pose(point[0],point[1],point[2])
            self.pose(pose_msg)
            time.sleep(0.3)

            self.conveyor_cmd_pubs.publish(Int32(data=-1)) # 컨베이어 80cm 이동

    def process3(self):
        '''
        1. 다시 앞을 바라보게 함
        2. 뒤로 이동 z가 1.2 이상이면 정지
        '''
        pose = PointToPose()
        point = POSE_DICT['look']
        pose_msg = pose.pose(point[0],point[1],point[2])
        self.pose(pose_msg)
        time.sleep(0.3)

        done = False
        while not done:
            z = round(self.robot_front_1_position[2],2)
            if z == 1.2:
                self.stop()
                done = True
            elif z > 1.2:
                self.go_front()
            elif z< 1.2:
                self.go_back()

    def process4(self):
        '''
        1. 보라색 박스를 찾기 위한 pose 
        2. 보라색 박스 결과 받기
        3. 보라색 박스의 위치 계산
        4. 보라색 박스의 위치로 이동
        5. 보라색 박스를 grip
        6. z축 방향으로 up (10cm)
        7. 
        '''
        pose = PointToPose()
        point = POSE_DICT['puple_box_detect']
        pose_msg = pose(point[0],point[1],point[2])
        self.pose(pose_msg)
        time.sleep(0.5) # 0.5초 대기

        boxes = self.boxes
        clss = self.clss
        index = clss.index('purple') # 보라색 박스의 인덱스
        box = boxes[index]
        x,y,z = self.yolo_pose(box,self.robot_front_1_position)
        pose = PointToPose()
        pose_msg = pose(x,y,z)
        self.pose(pose_msg)
        time.sleep(0.3)

        self.grip()
        time.sleep(0.1)

        pose_msg = pose(x,y,z+0.1) # 로봇의 팔을 움직이기 위한 메시지
        self.pose(pose_msg)
        time.sleep(0.3)

        point = POSE_DICT['Yolo_Box_Detect'] # 정면을 바라보기 위한 함수 
        pose_msg = pose(point[0],point[1],point[2])
        self.pose(pose_msg)
        time.sleep(0.3)
        
    def process5(self):
        '''
        
        '''
        pass
    
    def process6(self):
        pass
    def process7(self):
        pass
    def process8(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControllerTower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
