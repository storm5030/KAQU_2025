#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import serial
import threading
import time

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

# 다이나믹셀 SDK 라이브러리 임포트
from dynamixel_sdk import *

# ===================== 사용자 설정 영역 =====================

# 아두이노/OpenCR 보드가 연결된 시리얼 포트 이름
# Linux: "/dev/ttyACM0", "/dev/ttyUSB0" 등
# Windows: "COM3", "COM4" 등
SERIAL_PORT = '/dev/ttyACM0'
# 시리얼 포트 통신 속도 (아두이노 코드의 CMD_PORT와 일치시켜야 함)
SERIAL_BAUDRATE = 115200

# 다이나믹셀 프로토콜 및 제어 테이블 주소 (기존 bulk_read_write.py 참고)
PROTOCOL_VERSION = 2.0
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4

# 다이나믹셀 ID 리스트
DXL_ID_LIST = [
    11, 12, 13,  # FR
    21, 22, 23,  # FL
    31, 32, 33,  # RR
    41, 42, 43   # RL
]

# ==========================================================


class ArduinoBridgeNode(Node):
    """
    ROS 2 제어 시스템과 아두이노 하드웨어 브릿지 간의 양방향 통신을 담당하는 노드.
    - ROS 토픽으로 받은 관절 각도를 다이나믹셀 패킷으로 변환하여 전송 (Write-Only Passthrough).
    - 아두이노로부터 받은 IMU 데이터를 파싱하여 ROS 토픽으로 발행.
    """
    def __init__(self):
        super().__init__('arduino_bridge_node')
        self.get_logger().info("Arduino Bridge Node 시작 중...")

        # 1. 시리얼 포트 초기화
        try:
            self.serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1.0)
            self.get_logger().info(f"시리얼 포트 {SERIAL_PORT} 연결 성공.")
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 {SERIAL_PORT} 연결 실패: {e}")
            rclpy.shutdown()
            return

        # 2. 다이나믹셀 SDK 핸들러 초기화 (쓰기 전용)
        # PortHandler는 실제 포트가 아닌 가상 핸들러로 사용. 실제 전송은 pyserial이 담당.
        self.portHandler = PortHandler(SERIAL_PORT) 
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)

        # 3. ROS 퍼블리셔 및 서브스크라이버 설정
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.control_subscriber = self.create_subscription(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            self.control_callback,
            10)

        # 4. 시리얼 데이터 수신을 위한 별도 스레드 시작
        self.is_running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.get_logger().info("Arduino Bridge Node가 성공적으로 시작되었습니다.")

    def control_callback(self, msg):
        """서브스크라이버 콜백: 관절 각도(radian)를 받아 다이나믹셀 패킷으로 전송"""
        if len(msg.data) != len(DXL_ID_LIST):
            self.get_logger().warn(f"수신된 데이터 길이가 모터 개수와 다릅니다. (수신: {len(msg.data)}, 필요: {len(DXL_ID_LIST)})")
            return

        self.groupBulkWrite.clearParam()
        
        # 라디안 각도를 다이나믹셀 위치 값으로 변환하고 BulkWrite 파라미터에 추가
        for i, angle_rad in enumerate(msg.data):
            # 라디안 -> 0~4095 사이의 다이나믹셀 위치 값으로 변환
            # 참고: 이 변환은 모터의 0점 위치나 방향에 따라 달라질 수 있음
            # 예시: 0 rad = 2048, pi rad = 3072, -pi rad = 1024
            position = int(2048 + (angle_rad * (2048.0 / np.pi)))
            position = max(0, min(4095, position)) # 범위 제한

            dxl_id = DXL_ID_LIST[i]
            
            # 4바이트 위치 값을 바이트 배열로 변환
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(position)),
                DXL_HIBYTE(DXL_LOWORD(position)),
                DXL_LOBYTE(DXL_HIWORD(position)),
                DXL_HIBYTE(DXL_HIWORD(position))
            ]
            
            # BulkWrite 그룹에 파라미터 추가
            addparam_result = self.groupBulkWrite.addParam(dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
            if not addparam_result:
                self.get_logger().error(f"[ID:{dxl_id}] BulkWrite addParam 실패")
                return

        # BulkWrite 패킷 생성 (이진 데이터)
        tx_packet = self.groupBulkWrite.generatePacket()

        # 생성된 패킷을 시리얼 포트를 통해 아두이노로 직접 전송
        try:
            self.serial_port.write(tx_packet)
            # self.get_logger().info(f"모터 명령 전송: {len(tx_packet)} bytes")
        except serial.SerialException as e:
            self.get_logger().error(f"모터 명령 전송 실패: {e}")

    def serial_read_loop(self):
        """스레드 루프: 아두이노로부터 들어오는 IMU 데이터를 읽고 파싱하여 ROS 토픽으로 발행"""
        while self.is_running and rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()

                    if line.startswith('IMU,'):
                        parts = line.split(',')
                        if len(parts) == 5: # "IMU,micros,roll,pitch,yaw"
                            roll_deg = float(parts[2])
                            pitch_deg = float(parts[3])
                            yaw_deg = float(parts[4])

                            # 도(degree)를 라디안(radian)으로 변환
                            roll_rad = np.deg2rad(roll_deg)
                            pitch_rad = np.deg2rad(pitch_deg)
                            yaw_rad = np.deg2rad(yaw_deg)

                            # 오일러 각을 쿼터니언으로 변환
                            qx, qy, qz, qw = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

                            # Imu 메시지 생성 및 발행
                            imu_msg = Imu()
                            imu_msg.header.stamp = self.get_clock().now().to_msg()
                            imu_msg.header.frame_id = 'imu_link'  # URDF에 정의된 프레임 이름
                            imu_msg.orientation.x = qx
                            imu_msg.orientation.y = qy
                            imu_msg.orientation.z = qz
                            imu_msg.orientation.w = qw
                            
                            self.imu_publisher.publish(imu_msg)
            except Exception as e:
                self.get_logger().warn(f"시리얼 읽기 루프 에러: {e}")
            
            time.sleep(0.001) # CPU 사용량 완화

    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각(Roll, Pitch, Yaw)을 쿼터니언으로 변환"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw

    def destroy_node(self):
        """노드 종료 시 자원 정리"""
        self.get_logger().info("노드 종료 중... 시리얼 포트를 닫습니다.")
        self.is_running = False
        self.read_thread.join(timeout=1)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    
    # 노드가 성공적으로 초기화되었는지 확인
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("키보드 인터럽트로 종료")
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()