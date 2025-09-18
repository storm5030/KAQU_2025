#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU-based 2D pose estimator for ROS 2 Humble.
- Maintains x [m], y [m], yaw [deg], vx [m/s], vy [m/s]
- Consumes sensor_msgs/Imu
- Supports bias calibration, quaternion-based yaw, ZUPT, and simple velocity leak to reduce drift.

Author: kaqu project
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

try:
    from sensor_msgs.msg import Imu
except Exception:
    # Allow import without ROS env for static checks/tests
    Imu = object  # type: ignore


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """
    Compute yaw (Z) in degrees from quaternion (x, y, z, w).
    Uses standard ZYX Euler extraction.
    """
    # Yaw (psi)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


@dataclass
class PoseEstimatorConfig:
    # Calibration samples (assumed stationary) for bias estimation
    calib_samples: int = 200

    # If True, linear_acceleration includes gravity; for most IMU drivers this is False
    accel_includes_gravity: bool = False

    # Leak factor [1/s] for velocity drift suppression (0 ~ 0.2 typical)
    vel_leak_rate: float = 0.02

    # Use quaternion orientation for yaw if orientation is valid
    prefer_quat_yaw: bool = True

    # Clamp yaw to [-180, 180]
    wrap_yaw_deg: bool = True

    # Minimum dt [s] to avoid zero/negative time steps
    min_dt: float = 1e-3

    # Optional velocity clamp [m/s] to avoid blow-ups (0 disables)
    vel_abs_limit: float = 0.0

    # Optional accel clamp [m/s^2] to robustify against spikes (0 disables)
    accel_abs_limit: float = 0.0


class ImuPose2D:
    """
    Lightweight 2D pose estimator using IMU.
    - Call update(imu_msg) for each incoming IMU sample.
    - Read pose via get_pose().
    - Call reset() to clear state at new route start.
    - Optionally call apply_zupt(is_stance=True) in stance phase to reduce drift.
    """

    def __init__(self, cfg: Optional[PoseEstimatorConfig] = None):
        self.cfg = cfg if cfg is not None else PoseEstimatorConfig()
        self.reset()

    # ---------- Public API ----------

    def reset(self) -> None:
        self.x = 0.0      # [m]
        self.y = 0.0      # [m]
        self.yaw_deg = 0.0
        self.vx = 0.0     # [m/s]
        self.vy = 0.0     # [m/s]
        self._last_t: Optional[float] = None

        # Bias (estimated during calibration window)
        self._bias_ax = 0.0
        self._bias_ay = 0.0
        self._bias_gz = 0.0  # [rad/s]

        # Calibration
        self._calib_count = 0
        self.ready = False

    def update(self, imu: Imu) -> None:
        """
        Main update with a ROS2 Imu message.
        """
        # Timestamp to float seconds
        try:
            t = float(imu.header.stamp.sec) + float(imu.header.stamp.nanosec) * 1e-9
        except Exception:
            # If header missing, do nothing
            return

        if self._last_t is None:
            self._last_t = t
            # accumulate bias from the first sample too
            self._accumulate_bias(imu)
            return

        dt = max(self.cfg.min_dt, t - self._last_t)
        self._last_t = t

        # During calibration window, only accumulate bias then return
        if not self.ready:
            self._accumulate_bias(imu)
            return

        # Raw signals
        ax_b = float(imu.linear_acceleration.x)
        ay_b = float(imu.linear_acceleration.y)
        gz   = float(imu.angular_velocity.z)  # [rad/s]

        # Optional clamps for robustness
        if self.cfg.accel_abs_limit > 0.0:
            ax_b = max(-self.cfg.accel_abs_limit, min(self.cfg.accel_abs_limit, ax_b))
            ay_b = max(-self.cfg.accel_abs_limit, min(self.cfg.accel_abs_limit, ay_b))

        # Bias removal
        ax_b -= self._bias_ax
        ay_b -= self._bias_ay
        gz   -= self._bias_gz

        # Yaw update: prefer quaternion if available
        yaw_deg_meas: Optional[float] = None
        if self.cfg.prefer_quat_yaw and hasattr(imu, "orientation"):
            qx = float(imu.orientation.x)
            qy = float(imu.orientation.y)
            qz = float(imu.orientation.z)
            qw = float(imu.orientation.w)
            # If quaternion seems valid (norm ~ 1)
            if (qx*qx + qy*qy + qz*qz + qw*qw) > 1e-6:
                yaw_deg_meas = _yaw_from_quaternion(qx, qy, qz, qw)

        if yaw_deg_meas is None:
            # integrate gz if quat not available
            self.yaw_deg += math.degrees(gz) * dt
        else:
            # simple complementary fusion (alpha tunes trust in quat vs gyro integration)
            alpha = 0.9
            est = self._wrap_deg(self.yaw_deg) if self.cfg.wrap_yaw_deg else self.yaw_deg
            meas = self._wrap_deg(yaw_deg_meas) if self.cfg.wrap_yaw_deg else yaw_deg_meas
            # handle wrap-around softly
            diff = self._angle_diff_deg(meas, est)
            self.yaw_deg = est + alpha * diff

        if self.cfg.wrap_yaw_deg:
            self.yaw_deg = self._wrap_deg(self.yaw_deg)

        # Body -> World rotation (yaw only)
        yaw = math.radians(self.yaw_deg)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        ax_w = ax_b * cos_y - ay_b * sin_y
        ay_w = ax_b * sin_y + ay_b * cos_y

        # If acceleration includes gravity, (roll/pitch small) we assume xy channels mostly free of g
        # For flat ground and small roll/pitch this simplification is acceptable.

        # Velocity integrate + leak
        self.vx += ax_w * dt
        self.vy += ay_w * dt

        leak = max(0.0, float(self.cfg.vel_leak_rate))
        if leak > 0.0:
            self.vx *= (1.0 - leak * dt)
            self.vy *= (1.0 - leak * dt)

        # Optional velocity clamp
        if self.cfg.vel_abs_limit > 0.0:
            lim = self.cfg.vel_abs_limit
            self.vx = max(-lim, min(lim, self.vx))
            self.vy = max(-lim, min(lim, self.vy))

        # Position integrate
        self.x += self.vx * dt
        self.y += self.vy * dt

    def get_pose(self) -> Tuple[float, float, float]:
        """Returns (x_m, y_m, yaw_deg)."""
        return float(self.x), float(self.y), float(self.yaw_deg)

    def get_velocity(self) -> Tuple[float, float]:
        """Returns (vx, vy) in m/s."""
        return float(self.vx), float(self.vy)

    def apply_zupt(self, is_stance: bool, alpha: float = 0.5) -> None:
        """
        Zero-Velocity Update (optional): call with is_stance=True during stance/contact.
        alpha in [0,1]: 1.0 = hard reset to zero velocity
        """
        if not is_stance:
            return
        alpha = max(0.0, min(1.0, alpha))
        self.vx *= (1.0 - alpha)
        self.vy *= (1.0 - alpha)

    # ---------- Internal helpers ----------

    def _accumulate_bias(self, imu: Imu) -> None:
        """Accumulate mean bias from first N stationary samples."""
        ax_b = float(getattr(imu.linear_acceleration, "x", 0.0))
        ay_b = float(getattr(imu.linear_acceleration, "y", 0.0))
        gz   = float(getattr(imu.angular_velocity, "z", 0.0))

        self._bias_ax += ax_b
        self._bias_ay += ay_b
        self._bias_gz += gz
        self._calib_count += 1

        if self._calib_count >= int(self.cfg.calib_samples):
            n = float(self._calib_count)
            self._bias_ax /= n
            self._bias_ay /= n
            self._bias_gz /= n
            self.ready = True  # start integrating from next update

    @staticmethod
    def _wrap_deg(a: float) -> float:
        """Wrap angle to [-180, 180]."""
        while a > 180.0:
            a -= 360.0
        while a < -180.0:
            a += 360.0
        return a

    @staticmethod
    def _angle_diff_deg(target: float, source: float) -> float:
        """Shortest signed difference target - source in degrees (wrapped)."""
        diff = target - source
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff
