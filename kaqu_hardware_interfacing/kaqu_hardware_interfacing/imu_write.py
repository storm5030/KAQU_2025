#!/usr/bin/env python3
import time
import math

# smbus / smbus2 둘 다 대응
try:
    import smbus2 as smbus
except ImportError:
    import smbus

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ==============================
# MPU6050 / GY-521 설정
# ==============================
MPU6050_ADDR    = 0x68

PWR_MGMT_1      = 0x6B
SMPLRT_DIV      = 0x19
CONFIG          = 0x1A
GYRO_CONFIG     = 0x1B
ACCEL_CONFIG    = 0x1C

ACCEL_XOUT_H    = 0x3B
GYRO_XOUT_H     = 0x43

# I2C 버스 (라즈베리파이의 I2C-1 사용)
bus = smbus.SMBus(1)


def write_reg(reg, value):
    bus.write_byte_data(MPU6050_ADDR, reg, value)


def read_word(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low  = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    return value


def read_word_2c(reg):
    val = read_word(reg)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


def mpu6050_init():
    # 슬립 모드 해제
    write_reg(PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # 샘플 레이트 = 8kHz / (1 + SMPLRT_DIV)
    write_reg(SMPLRT_DIV, 0x07)

    # DLPF 설정 (bandwidth 설정, 여기서는 0x06 정도)
    write_reg(CONFIG, 0x06)

    # 자이로 풀스케일 범위 ±250 deg/s
    write_reg(GYRO_CONFIG, 0x00)

    # 가속도 풀스케일 범위 ±2g
    write_reg(ACCEL_CONFIG, 0x00)


def read_accel():
    ax_raw = read_word_2c(ACCEL_XOUT_H)
    ay_raw = read_word_2c(ACCEL_XOUT_H + 2)
    az_raw = read_word_2c(ACCEL_XOUT_H + 4)

    # ±2g 범위에서 LSB/g = 16384
    ax = ax_raw / 16384.0
    ay = ay_raw / 16384.0
    az = az_raw / 16384.0
    return ax, ay, az


def read_gyro():
    gx_raw = read_word_2c(GYRO_XOUT_H)
    gy_raw = read_word_2c(GYRO_XOUT_H + 2)
    gz_raw = read_word_2c(GYRO_XOUT_H + 4)

    # ±250 deg/s 범위에서 LSB/(deg/s) = 131
    gx = gx_raw / 131.0
    gy = gy_raw / 131.0
    gz = gz_raw / 131.0
    return gx, gy, gz


def euler_to_quaternion(roll, pitch, yaw):
    """
    roll, pitch, yaw [rad] → quaternion (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class Mpu6050ImuNode(Node):
    def __init__(self):
        super().__init__('mpu6050_imu_node')

        # ROS2 퍼블리셔
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)

        # 파라미터 (필요하면 나중에 declare_parameter로 바꿔도 됨)
        self.alpha = 0.98    # Complementary filter 계수
        self.frame_id = 'imu_link'

        # MPU6050 초기화
        self.get_logger().info('Initializing MPU6050...')
        mpu6050_init()
        self.get_logger().info('MPU6050 init done.')

        # 초기 roll/pitch (가속도 기반)
        ax, ay, az = read_accel()
        self.roll_deg  = math.degrees(math.atan2(ay, az))
        self.pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
        self.yaw_deg   = 0.0  # 자이로 적분 시작값

        self.prev_time = time.time()

        # 0.02초(50Hz) 타이머
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()  # [deg/s]

        # 1) 가속도 기반 roll/pitch 계산 (deg)
        acc_roll_deg  = math.degrees(math.atan2(ay, az))
        acc_pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        # 2) 자이로 적분 (deg/s * s = deg)
        gyro_roll_deg  = self.roll_deg  + gx * dt
        gyro_pitch_deg = self.pitch_deg + gy * dt
        gyro_yaw_deg   = self.yaw_deg   + gz * dt   # yaw는 드리프트 있음

        # 3) Complementary filter
        self.roll_deg  = self.alpha * gyro_roll_deg  + (1.0 - self.alpha) * acc_roll_deg
        self.pitch_deg = self.alpha * gyro_pitch_deg + (1.0 - self.alpha) * acc_pitch_deg
        self.yaw_deg   = gyro_yaw_deg   # yaw는 보정 센서 없으니 그대로 사용

        # ROS2 Imu 메시지 생성
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # roll, pitch, yaw [deg] → [rad]
        roll_rad  = math.radians(self.roll_deg)
        pitch_rad = math.radians(self.pitch_deg)
        yaw_rad   = math.radians(self.yaw_deg)

        # orientation: quaternion
        qx, qy, qz, qw = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # angular_velocity: [rad/s] (자이로 원본 사용)
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        # linear_acceleration: [m/s^2] (가속도 원본 사용)
        g = 9.80665
        msg.linear_acceleration.x = ax * g
        msg.linear_acceleration.y = ay * g
        msg.linear_acceleration.z = az * g

        # covariance는 일단 모른다는 의미로 -1 (기본값) 유지
        # msg.orientation_covariance[0] = -1.0

        self.pub_imu.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
