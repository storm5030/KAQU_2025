#!/usr/bin/env python
# -*- coding: utf-8 -*-

# bulk_read_write
# 계산하는 애들로부터 값을 받아서 모터에 보내는 코드
# 테스트 방식
# 1)쓰기 비활성화 -> 읽기만 해서(모터에 토크 끄고) : 모니터에 띄워서 말이 되는 각도인지 확인
#   쓰기는 따로(나머지 내용 다 지우고 저 쓰는 부분만 남겨서(timer callback만 남겨서))
#   아예 링크 분해해 놓고/모터만 있는 상태에서 
# 2)말이 되는 각도를 보내서 -> 쓰기 -> 모터 작동 확인

# TODO list
# 1. USB 시리얼 번호 찾기

# Tuning
# 움직이기 위한 최소 각도 차이 : DXL_MOVING_STATUS_THRESHOLD
# IK 풀때 에러 : IK_ERROR_RANGE
# timer period

import os
import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin, tan, atan2, acos, sqrt, pi
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu 

# 이부분 지피티에게 물어보니 OS가 뭔지 판단하는 부분이라고 함
import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin, tan, atan2, acos, sqrt, pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu 

# 이부분 지피티에게 물어보니 OS가 뭔지 판단하는 부분이라고 함
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

#os.sys.path.append('./../../DynamixelSDK/python/src/dynamixel_sdk/dynamixel_functions_py')             # Path setting
from dynamixel_sdk import *  #다이나믹셀 라이브러리 사용

MY_DXL = 'X_SERIES'  

# Control table address
# 다른 변수가 필요한 경우 e-manual로부터 여기에 적어놓고 시작
# 다른 변수가 필요한 경우 e-manual로부터 여기에 적어놓고 시작
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_LED_RED                = 65
    LEN_LED_RED                 = 1         # Data Byte Length

    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length

    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    
    BAUDRATE                    = 2000000
    
    ADDR_OPERATING_MODE         = 11
    LEN_OPERATING_MODE          = 1
    DEFAULT_POS_MODE            = 3
    EXTENDED_POS_MODE           = 4         # 우리가 쓸 모드

    ADDR_HOMING_OFFSET          = 20        # 호밍 오프셋
    LEN_HOMING_OFFSET           = 4

    ADDR_DRIVE_MODE             = 10        # 주의 : 이거 수작업으로 가능한데 수작업으로 일단 해봅시다

    # 포지션 제어 개인
    ADDR_POS_D_GAIN = 80
    ADDR_POS_I_GAIN = 82
    ADDR_POS_P_GAIN = 84

    # Set Velocity Limit to 600 (approx. 137.4 RPM) 기본값 200정도
    VELOCITY_LIMIT_VALUE = 1000
    ADDR_VELOCITY_LIMIT = 44
    LEN_VELOCITY_LIMIT = 4

# 아래 내용은 지워버리거나 각 모터별로 다르게 적용하면 될 듯. 
DXL_MINIMUM_POSITION_VALUE  = -1024         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  =  1024         # Refer to the Maximum Position Limit of product eManual
DXL_2PI                     = 4095          # 포지션 기준 한바퀴. 앞으로 변환할 때는 이 값을 씁시다

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

# DYNAMIXEL Protocol Version 
PROTOCOL_VERSION            = 2.0

# Default setting
FR1_ID                     = 11                           
FR2_ID                     = 12                           
FR3_ID                     = 13
FL1_ID                     = 21
FL2_ID                     = 22
FL3_ID                     = 23
RR1_ID                     = 31
RR2_ID                     = 32
RR3_ID                     = 33
RL1_ID                     = 41
RL2_ID                     = 42
RL3_ID                     = 43

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyACM0'
IK_ERROR_RANGE = 0.1

# 주의 : 포지션 모드 확장 모드로 변경 필요

dxl_led_value = [0x00, 0x01]                                                        # Dynamixel LED value for write
dxl_id = [FR1_ID, FR2_ID, FR3_ID]
# dxl_id = [FR1_ID, FR2_ID, FR3_ID, FL1_ID, FL2_ID, FL3_ID, RR1_ID, RR2_ID, RR3_ID, RL1_ID, RL2_ID, RL3_ID]

# 계산에 필요한 하드웨어 스펙
# 이 부분을 코드에 박아둘까요 말까요
l1 = 100.0
l2 = 36.0
l3 = 130.0
l4a = 36.0
lhip = 31.5



angle_reverse = [1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1]
#camber = 60
camber = 0
dxl_offset = [2048, 0-camber, 0+camber, 2048, 4095+camber, 4095-camber, 2048, 0, 0, 2048, 4095, 4095] # sim->real 방향 기준으로 + 해주면 됨

# 원하는 게인 값
# P게인: 현재 값보다 높게 설정 (기본 800)
# I게인: 보통 0 사용
# D게인: 진동 잡을 때 사용 (보통 P게인의 1/10 ~ 1/20 수준이나 0부터 시작)
MY_P_GAIN = 3000 
MY_I_GAIN = 0
MY_D_GAIN = 10

# Initialize PortHandler, PacketHandler instance
# Initialize GroupBulkWrite instance / Initialize GroupBulkRead instace for Present Position
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    getch()
    quit()

print(f"Setting P-Gain to {MY_P_GAIN}...")

# 컨트롤 모드 변경
for i in dxl_id:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, EXTENDED_POS_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode changed to extended position control mode of dxl No. ", i)

    # Velocity Limit 수정
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, i, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print("[ID:%03d] Velocity Limit write failed: %s" % (i, packetHandler.getTxRxResult(dxl_comm_result)))
    elif dxl_error != 0:
        print("[ID:%03d] Velocity Limit error: %s" % (i, packetHandler.getRxPacketError(dxl_error)))
    else:
        print("[ID:%03d] Velocity Limit set to %d" % (i, VELOCITY_LIMIT_VALUE))

    # 모터에 게인 값 쓰기 (반복문)
    # (1) P Gain 설정 (2 Byte)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, i, ADDR_POS_P_GAIN, MY_P_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"[ID:{i:03d}] Failed to set P Gain: {packetHandler.getTxRxResult(dxl_comm_result)}")
    
    # (2) I Gain 설정 (필요하면 주석 해제)
    # packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_POS_I_GAIN, MY_I_GAIN)
    
    # (3) D Gain 설정 (필요하면 주석 해제)
    # packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_POS_D_GAIN, MY_D_GAIN)

print("Gain setting complete.")



# 리버스 모터 설정
# 주의 : 일단 이거는 그냥 수동으로 하겠습니다


# present position에 대한 parameter 저장소 추가
for i in dxl_id:
    dxl_addparam_result = groupBulkRead.addParam(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % i)
        quit()

dxl_comm_result = groupBulkRead.txRxPacket()
if dxl_comm_result != COMM_SUCCESS:
    print(f"{packetHandler.getTxRxResult(dxl_comm_result)}")


# 각 모터 토크 켜기. 이 때 모터가 살짝 움직이게 될 것. 
for i in range(len(dxl_id)):
    dxl_comm_result, dxl_error= packetHandler.write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        quit()
    else:
        print("Dynamixel#%d has been successfully connected" % dxl_id[i])

    # dxl_getdata_result = groupBulkRead.isAvailable(dxl_id[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    # if dxl_getdata_result != True:
    #     print("[ID:%03d] groupBulkRead getdata failed" % dxl_id[i])
    #     quit()

# 여기까지 초기 세팅

# Bulk_Read_Write 노드
class Bulk_Read_Write(Node):
    def __init__(self, last_cmd):
        super().__init__('bulk_read_write')

        # 이 내용은 지속적으로 업데이트 되는 내용이라 내부변수로 뒀습니당
        # self.dxl_goal_position = dxl_goal_position
        
        self.dxl_id = dxl_id
        self.portHandler = portHandler
        self.packetHandler = packetHandler
        self.groupBulkWrite = groupBulkWrite
        self.groupBulkRead = groupBulkRead

        self.last_command = last_cmd
        self.goal_position = [0]*12 #엉덩이, 없애도 될듯?
        self.last_read_joint_dxl = [0]*12 #현재
        self.last_read_sensor = [0]*3 # 일단 뭐가 될지 모르겠는데 배열 형태로 보내면 어떨s까

        # 주의 : openCR 연결해보고 timer period 조절해야 함
        data_pub_period = 0.02
        control_period = 0.02

        # 다리 각도 제어값(엉덩이)
        # 주의 : msg타입, 토픽이름 수정해야 함. 
        self.control_subscriber = self.create_subscription(Float64MultiArray, 'joint_group_position_controller/commands', self.control_callback, 20)
        self.present_angle_publisher = self.create_publisher(Float64MultiArray, 'real_leg_angle', 20)
        # self.imu_data_publisher = self.create_publisher(Imu, 'imu_data', 20)
        
        # ROS로 현재 상황을 보내는 퍼블리셔
        self.pos_timer = self.create_timer(data_pub_period, self.publish_data)
        
        # 로봇으로 데이터를 보내는 퍼블리셔
        self.robot_timer = self.create_timer(control_period, self.timer_callback)


    # 다리 각도 제어값(A1) -> goal position
    def control_callback(self, msg):
        # msg 받는곳 주의
        cmd_angle = msg.data
        # cmd_vel = msg.velocity

        # print(cmd_angle)

        # Transform
        goal_pos = self.sim_to_real_transform(cmd_angle)
        # print(goal_pos)
        
        # last command update(다이나믹셀 각도)
        # 주의 : valid 여부 다르게 해야함
        for i in range(len(goal_pos)):
            if goal_pos[i]<0:
                print("something wrong with transform")
                return
            else:
                pass
        self.last_command = goal_pos

    # ROS로 다리 각도 실제값 (A2), 센서값 (B) 발행
    # 주의 : 함수 구조 살펴봐야 함. 
    def publish_data(self):
        pass
        angle_msg = Float64MultiArray()
        # imu_msg = Imu()

        # # 읽은 값 받아오기
        real_angle = self.real_to_sim_transform(self.last_read_joint_dxl)

        # # 모터값 라디안으로 변환하여 넣기
        for i in range(len(self.last_read_joint_dxl)):
            angle_msg.data.append(real_angle[i])

        # 받아온 센서값 넣기
        
        # 퍼블리시
        self.present_angle_publisher.publish(angle_msg)
        # self.imu_data_publisher.publish(imu_msg)

    def timer_callback(self):
        # 읽기 먼저
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        print("-------------------------------------------------------")
        print(" ID  |  Goal Pos  |  Present Pos  |  Error (Goal-Pres)")
        print("-------------------------------------------------------")

        for i, motor_id in enumerate(dxl_id):
            # (1) Present Position 데이터 가져오기
            # getData(ID, Address, Length)
            dxl_present_position = groupBulkRead.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            self.last_read_joint_dxl[i] = dxl_present_position

            # (2) Goal Position (사용자가 쓰려고 준비한 변수)
            # param_goal_position 리스트가 바이트 배열이라면, 정수로 변환해서 봐야 합니다.
            # 만약 정수형 목표값 변수(예: goal_pos_list[i])가 따로 있다면 그걸 출력하세요.
            # 여기서는 예시로 goal_position 변수를 사용한다고 가정합니다.
            current_goal_pos = self.last_command[i] # <-- 님 코드의 목표 위치 변수

            # (3) 오차 계산
            error = current_goal_pos - dxl_present_position

            # (4) 출력
            print(f" {motor_id:03d} | {current_goal_pos:10d} | {dxl_present_position:13d} | {error:10d}")
            
            # [진단 Tip]
            # Error가 양수(+)면: 목표가 더 큼 -> 덜 올라갔거나(처짐), 아직 가는 중
            # Error가 0에 가까움: 잘 도달함
        # for i in range(len(dxl_id)):
        #     dxl_getdata_result = self.groupBulkRead.isAvailable(dxl_id[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        #     if dxl_getdata_result != True:
        #         print("[ID:%03d] groupBulkRead getdata failed" % dxl_id[i])
        #         # quit()
        #         return
            # present pos 가져오기
            # self.last_read_joint_dxl[i] = self.groupBulkRead.getData(dxl_id[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            # print("[ID:%03d] Present Position : %d" % (dxl_id[i], self.last_read_joint_dxl[i]))
        
        # 쓰기 
        # for i in range(len(self.last_command)):
        for i in range(len(dxl_id)):
            # 마지막 명령값 받아오기
            # print("last command : ", self.last_command[i])
            # self.goal_position[i] = self.last_command[i]
            # byte 단위의 배열로 쪼개기
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.last_command[i])), 
                                   DXL_HIBYTE(DXL_LOWORD(self.last_command[i])), 
                                   DXL_LOBYTE(DXL_HIWORD(self.last_command[i])), 
                                   DXL_HIBYTE(DXL_HIWORD(self.last_command[i]))]
            # Bulkwrite 파라미터 저장소에 추가
            dxl_addparam_result = self.groupBulkWrite.addParam(dxl_id[i], ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id[i])
                return
            
            # BulkWrite Goal Position
            dxl_comm_result = self.groupBulkWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" %self.packetHandler.getTxRxResult(dxl_comm_result))
        # 파라미터 저장소 비우기
        self.groupBulkWrite.clearParam()

    def sim_to_real_transform(self, cmd_angle):
        real_angle = [0]*12
        # cmd angle은 각도 반전이 된 각이 들어옴(시뮬에서도 그 각을 쓰니까)
        # 근데 Ik를 한 식으로 풀기 위해서는 아래의 과정을 거처야 함.
        # 반전됨 -> 반전안됨 -> IK 풀기 -> 반전됨 -> 모터에 전송

        # IK 풀기
        for i in range(4):
            roll = cmd_angle[3*i]
            alpha = cmd_angle[3*i+1]+pi
            beta1 = pi-(- cmd_angle[3*i+2] - cmd_angle[3*i+1])
            _a = -l1*cos(alpha)-l4a*cos(beta1)
            _b = lhip-l4a*sin(beta1)-l1*sin(alpha)
            _c = (l3**2-l2**2-_a**2-_b**2)/(2*l2)

            # 판별식
            if (_b**2-_c**2+_a**2)<0:
                print("something wrong with %03d th leg IK" %(i+1))
                break
            else:
                pass

            beta2 = 2*atan2(_b+sqrt(_b**2-_c**2+_a**2),(_a+_c))

            # 말이 되는 각도인지 확인
            if (l2*cos(beta2)-l4a*cos(beta1)-l1*cos(alpha))<(-IK_ERROR_RANGE):
                beta2 = 2*atan2(_b+sqrt(_b**2-_c**2+_a**2),(_a+_c))
                if (l2*cos(beta2)-l4a*cos(beta1)-l1*cos(alpha))<(-IK_ERROR_RANGE):
                    print("something wrong with %03d th leg IK" %(i+1))
                    break
                else: 
                    pass
            else:
                pass
            # 각도 넣기(라디안->다이나믹셀 각도)
            # 주의 : 모터 설치 각도를 고려해야 함. 
            real_angle[3*i] = int(roll*DXL_2PI/(2*pi))
            real_angle[3*i+1] = int(alpha*DXL_2PI/(2*pi))
            real_angle[3*i+2] = int(beta2*DXL_2PI/(2*pi))
        for i in range(len(dxl_id)):
            real_angle[i] = angle_reverse[i]*real_angle[i] + dxl_offset[i] # 다시 각도 반전 모터 방향에 따른 오프셋 적용

        return real_angle


    # 각도를 -pi ~ pi 사이로 변환
    def angle_normalize(self, x):
        return ((x + pi) % (2 * pi)) - pi

    def real_to_sim_transform(self, present_angle):
        rad_angle = [0]*12
        sim_angle = [0]*12

        # 1) 각도 변환 (오프셋 및 방향 처리)
        for i in range(len(dxl_id)):
            angle_without_offset = (present_angle[i] - dxl_offset[i]) * angle_reverse[i]
            rad_angle[i] = angle_without_offset * 2 * pi / DXL_2PI
        
        # 2) IK (Forward Kinematics)
        for i in range(4):
            # [입력값]
            roll = rad_angle[3*i]
            alpha = rad_angle[3*i+1] # Hip Pitch (Motor)
            beta2 = rad_angle[3*i+2] # Knee Pitch (Motor)

            # [Hip 복원] 
            # sim_to_real: alpha = hip_cmd + pi
            # 역산: hip_cmd = alpha - pi
            original_hip = alpha - pi
            
            # [Hip 오차 확인용 - 디버깅 시에만 주석 해제]
            # print(f"Hip Calc: {original_hip:.4f}") 

            # [Beta1(Linkage) 복원]
            _a = l2*cos(beta2)-l1*cos(alpha)
            _b = lhip+l2*sin(beta2)-l1*sin(alpha)
            _c = -(l3**2-l4a**2-_a**2-_b**2)/(2*l4a)
            
            discriminant = _b**2-_c**2+_a**2
            if discriminant < -0.0001: 
                print(f"Leg {i} Kinematics Error")
                sim_angle[3*i+2] = 0 # 에러 시 0도 처리
                continue
            if discriminant < 0: discriminant = 0 # 부동소수점 보정

            # [중요] beta1 계산
            # sim_to_real에서 +sqrt를 썼다면, 구조에 따라 여기서 -sqrt가 맞을 수도, +sqrt가 맞을 수도 있음
            # 일단 -sqrt로 두고, 결과가 이상하면 +sqrt로 변경 테스트 필요
            beta1 = 2*atan2(_b+sqrt(discriminant), _a+_c)

            # [Knee 복원]
            # 수식: knee = beta1 - pi - original_hip
            temp_knee = beta1 - pi - original_hip

            # 3) 최종 값 대입 및 보정
            sim_angle[3*i] = self.angle_normalize(roll)
            sim_angle[3*i+1] = self.angle_normalize(original_hip)

            # [문제 해결 파트]
            # 현상: 부호가 반대이고 절대값이 큼.
            # 조치 1: 부호를 반대로 뒤집음 (-temp_knee)
            # 조치 2: 절대값 오차는 'beta1' 계산의 sqrt 부호 문제일 수 있음.
            
            sim_angle[3*i+2] = self.angle_normalize(temp_knee) 

        return sim_angle

def main(args=None):
    rclpy.init(args=args)
    pos_init = [2048, 2595, 1595, 2048, 1500, 2500, 2048, 2595, 1595, 2048, 1500, 2500]
    node = Bulk_Read_Write(pos_init)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Bulk_Read_Write Node...")
    finally:
        node.portHandler.closePort()
        rclpy.shutdown()

        # Disable Dynamixel Torque
        for i in dxl_id:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
        # Close port
        portHandler.closePort()
    
if __name__ == '__main__':
    main()


