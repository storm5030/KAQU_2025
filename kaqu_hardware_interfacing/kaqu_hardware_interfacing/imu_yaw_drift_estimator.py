#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

def quat_to_rpy(qx, qy, qz, qw):
    # Standard aerospace sequence: roll (x), pitch (y), yaw (z)
    # roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class ImuYawDriftEstimator(Node):
    def __init__(self):
        super().__init__('imu_yaw_drift_estimator')

        # Parameters
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('measure_duration_s', 60.0)  # 추천: 60~180초
        self.declare_parameter('assumed_pub_hz', 50.0)      # 너가 말한 50Hz

        self.imu_topic = self.get_parameter('imu_topic').value
        self.T = float(self.get_parameter('measure_duration_s').value)
        self.hz = float(self.get_parameter('assumed_pub_hz').value)

        self.t0 = None
        self.ts = []
        self.yaws = []

        self.sub = self.create_subscription(Imu, self.imu_topic, self.on_imu, 50)
        self.timer = self.create_timer(1.0, self.on_timer)

        self.get_logger().info(
            f"Measuring yaw drift for {self.T:.1f}s on topic '{self.imu_topic}'..."
            " Keep the robot IMU as still as possible."
        )

    def stamp_to_sec(self, msg: Imu) -> float:
        # Use msg header stamp if valid; else fallback to node clock
        s = msg.header.stamp.sec
        ns = msg.header.stamp.nanosec
        if s == 0 and ns == 0:
            return self.get_clock().now().nanoseconds * 1e-9
        return float(s) + float(ns) * 1e-9

    def on_imu(self, msg: Imu):
        t = self.stamp_to_sec(msg)
        if self.t0 is None:
            self.t0 = t

        q = msg.orientation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

        self.ts.append(t - self.t0)
        self.yaws.append(yaw)

    def on_timer(self):
        if self.t0 is None:
            return
        elapsed = self.ts[-1] if self.ts else 0.0
        self.get_logger().info(f"elapsed={elapsed:.1f}s, samples={len(self.ts)}")

        if elapsed >= self.T and len(self.ts) >= 20:
            self.compute_and_print()
            self.get_logger().info("Done. You can Ctrl+C now.")
            # Stop further compute spam
            self.destroy_timer(self.timer)

    def compute_and_print(self):
        t = np.array(self.ts, dtype=float)
        yaw = np.array(self.yaws, dtype=float)

        # Unwrap yaw to avoid +-pi jumps
        yaw_u = np.unwrap(yaw)

        # Linear regression yaw_u ≈ a*t + b
        # a = rad/s drift rate
        a, b = np.polyfit(t, yaw_u, 1)

        drift_rad_s = float(a)
        drift_deg_s = float(a * 180.0 / math.pi)

        dt = 1.0 / max(1e-9, self.hz)
        drift_rad_per_sample = drift_rad_s * dt
        drift_deg_per_sample = drift_deg_s * dt

        # Basic sanity checks
        # If robot truly still, angular_velocity.z mean should have similar sign as drift
        self.get_logger().info("=== Yaw Drift Estimate ===")
        self.get_logger().info(f"drift_rate = {drift_rad_s:.6e} rad/s  ({drift_deg_s:.6e} deg/s)")
        self.get_logger().info(f"@{self.hz:.1f} Hz -> {drift_rad_per_sample:.6e} rad/sample ({drift_deg_per_sample:.6e} deg/sample)")
        self.get_logger().info("Interpretation: if drift_rate > 0, yaw increases over time (one direction).")

def main():
    rclpy.init()
    node = ImuYawDriftEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
