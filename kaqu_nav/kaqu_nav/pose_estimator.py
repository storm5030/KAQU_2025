#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
간단 IMU 2D 포즈 추정기 (최소 + 정확도 개선)
- yaw: 쿼터니언이 유효하면 사용, 아니면 gz 적분
- 바디 가속도 -> 월드 가속도 변환: 쿼터니언 전체 회전행렬 사용
- accel_has_gravity=True면 월드 z축에서 g 제거(중력 포함 IMU용)
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

try:
    from sensor_msgs.msg import Imu
except Exception:
    Imu = object  # type: ignore


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

def _quat_to_rotm(x: float, y: float, z: float, w: float):
    """단위쿼터니언 -> 3x3 회전행렬 (월드 = R * 바디)"""
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return ((1.0,0.0,0.0),(0.0,1.0,0.0),(0.0,0.0,1.0))
    x/=n; y/=n; z/=n; w/=n
    xx,yy,zz = x*x, y*y, z*z
    xy,xz,yz = x*y, x*z, y*z
    wx,wy,wz = w*x, w*y, w*z
    r00 = 1.0 - 2.0*(yy+zz); r01 = 2.0*(xy - wz);   r02 = 2.0*(xz + wy)
    r10 = 2.0*(xy + wz);     r11 = 1.0 - 2.0*(xx+zz); r12 = 2.0*(yz - wx)
    r20 = 2.0*(xz - wy);     r21 = 2.0*(yz + wx);     r22 = 1.0 - 2.0*(xx+yy)
    return ((r00,r01,r02),(r10,r11,r12),(r20,r21,r22))


@dataclass
class PoseEstimatorConfig:
    calib_samples: int = 100
    min_dt: float = 1e-3
    prefer_quat_yaw: bool = True
    accel_has_gravity: bool = False   # True면 월드 z에서 g 제거


class ImuPose2D:
    def __init__(self, cfg: Optional[PoseEstimatorConfig] = None):
        self.cfg = cfg if cfg is not None else PoseEstimatorConfig()
        self.reset()

    # ----------- Public ----------- #
    def reset(self) -> None:
        self.x = 0.0; self.y = 0.0; self.yaw_deg = 0.0
        self.vx = 0.0; self.vy = 0.0
        self._last_t: Optional[float] = None
        self._bx = 0.0; self._by = 0.0; self._bgz = 0.0
        self._n = 0
        self.ready = False

    def update(self, imu: Imu) -> None:
        # 시간
        try:
            t = float(imu.header.stamp.sec) + float(imu.header.stamp.nanosec) * 1e-9
        except Exception:
            return
        if self._last_t is None:
            self._last_t = t
            self._accumulate_bias(imu)
            return
        dt = max(self.cfg.min_dt, t - self._last_t)
        self._last_t = t

        # 캘리브레이션
        if not self.ready:
            self._accumulate_bias(imu)
            return

        # 바이어스 제거
        ax_b = float(imu.linear_acceleration.x) - self._bx
        ay_b = float(imu.linear_acceleration.y) - self._by
        gz   = float(imu.angular_velocity.z)   - self._bgz  # [rad/s]

        # yaw: quat 우선
        qx = float(getattr(imu.orientation, "x", 0.0))
        qy = float(getattr(imu.orientation, "y", 0.0))
        qz = float(getattr(imu.orientation, "z", 0.0))
        qw = float(getattr(imu.orientation, "w", 1.0))
        if self.cfg.prefer_quat_yaw and (qx*qx+qy*qy+qz*qz+qw*qw) > 1e-6:
            self.yaw_deg = _yaw_from_quaternion(qx, qy, qz, qw)
        else:
            self.yaw_deg += math.degrees(gz) * dt
        self.yaw_deg = self._wrap_deg(self.yaw_deg)

        # 바디 -> 월드 (풀 3D 회전)
        R = _quat_to_rotm(qx, qy, qz, qw)  # 월드 = R * 바디
        ax_w = R[0][0]*ax_b + R[0][1]*ay_b + R[0][2]*0.0
        ay_w = R[1][0]*ax_b + R[1][1]*ay_b + R[1][2]*0.0
        az_w = R[2][0]*ax_b + R[2][1]*ay_b + R[2][2]*0.0

        # (옵션) 중력 보정: 가제보/드라이버가 중력 포함으로 주면 True로
        if self.cfg.accel_has_gravity:
            az_w -= 9.80665

        # 적분(x,y만 사용)
        self.vx += ax_w * dt
        self.vy += ay_w * dt
        self.x  += self.vx * dt
        self.y  += self.vy * dt

    def get_pose(self) -> Tuple[float, float, float]:
        return float(self.x), float(self.y), float(self.yaw_deg)

    # ----------- Internal ----------- #
    def _accumulate_bias(self, imu: Imu) -> None:
        self._bx += float(getattr(imu.linear_acceleration, "x", 0.0))
        self._by += float(getattr(imu.linear_acceleration, "y", 0.0))
        self._bgz += float(getattr(imu.angular_velocity, "z", 0.0))
        self._n += 1
        if self._n >= int(self.cfg.calib_samples):
            n = float(self._n)
            self._bx /= n; self._by /= n; self._bgz /= n
            self.ready = True

    @staticmethod
    def _wrap_deg(a: float) -> float:
        if a > 180.0: a -= 360.0
        if a < -180.0: a += 360.0
        return a
