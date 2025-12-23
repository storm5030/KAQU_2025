#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS 2 Humble / Python3.10+
"""
IMU 전용 FollowPath 액션 서버(개선판)
핵심 개선:
 1) 전진 스텝의 진행도 = 유클리드 거리(hypot)로 측정 → 가속도 드리프트의 투영 오류 완화
 2) 회전 스텝 = 2단계 제어(빠르게 돌다가 근접 구간에서만 P제어) → 초반부터 충분히 빠르고, 마지막만 정밀
 3) 전역 헤딩 목표(yaw_target_deg)로 관리 → 턴 잔오차는 다음 전진 스텝의 헤딩-홀드로 자연 보정
 4) 턴 동안 ZUPT 적용(속도=0 가정) → 드리프트 축적 완화
"""


# 헤딩을 반대로 하기 위해서, execute_cb 내부의 lin_sign 및 ang 에 -1 곱함

import json
import time
import math
from typing import List, Dict, Any
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Joy, Imu
from kaqu_msgs.action import FollowPath

from kaqu_nav.pose_estimator import ImuPose2D, PoseEstimatorConfig

from kaqu_controller.KaquCmdManager.KaquParams import LegParameters

class FollowPathServer(Node):
    def __init__(self):
        super().__init__('follow_path_server')

        # [필수 IO] 조이스틱 퍼블리셔
        self.joy_topic = '/joy'
        self.idx_lin = 4
        self.idx_yaw = 6
        self.joy_pub = self.create_publisher(Joy, self.joy_topic, 10)

        # [제어 주기]
        self.pub_hz = 50
        self.dt = 1.0 / self.pub_hz


        leg_params = LegParameters()     
        trot = leg_params.gait 

        x_vel_gain = 0.8;

        # 전진 속도 [m/s] (양수)
        self.x_vel = trot.max_x_vel * 4 / x_vel_gain * 0.001 # mm/s -> m/s 변환
        # yaw 속도 [deg/s] (양수)
        self.yaw_rate_deg_s = np.degrees(trot.max_yaw_rate)
        self.turn_fast_window_deg = 12.0
        self.turn_fast_axis = 0.8     # 빠른 구간 속도
        self.turn_slow_axis = 0.2     # 근접 구간(절반 속도 고정) ★비례제어 제거
        self.yaw_tol_deg = 3.0

        # 전진 헤딩 P
        self.kp_yaw = 0.10
        self.min_yaw_axis = 0.10

        # 전진 종료
        self.dist_tol = 0.03
        self.lin_axis_mag = 1.0
        self.max_step_time_s = 60

        # [IMU + QoS]
        cfg = PoseEstimatorConfig(
            calib_samples=100,
            prefer_quat_yaw=True,
            accel_has_gravity=True  # 가제보 IMU가 중력 포함이면 True로 바꾸세요
        )
        self.imu_est = ImuPose2D(cfg)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )
        self.create_subscription(Imu, '/imu', self._on_imu, sensor_qos)

        # [전역 헤딩 목표] — 모든 스텝에서 동일한 기준으로 보정
        self.yaw_target_deg = None

        # 액션 서버
        self._action_server = ActionServer(
            self, FollowPath, 'follow_path',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        # (디버그) IMU 수신률 지표
        self._imu_rx = 0
        self.create_timer(1.0, self._imu_watchdog)

        self.get_logger().info('FollowPathServer (IMU-only, improved) started.')

    # ----------------- 액션 콜백 ----------------- #

    def goal_cb(self, goal_request: FollowPath.Goal) -> GoalResponse:
        try:
            steps = self._parse_route_json(goal_request.route_json)
        except Exception as e:
            self.get_logger().warn(f'Invalid route_json: {e}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if steps else GoalResponse.REJECT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        self._stop_joy()
        self.imu_est.reset()

        # IMU 준비 대기(간단)
        if not self._wait_imu_ready(timeout_s=3.0):
            msg = 'IMU not ready.'
            self.get_logger().error(msg)
            goal_handle.abort()
            return FollowPath.Result(success=False, message=msg,
                                     total_time_s=0.0, final_x_m=0.0, final_y_m=0.0, final_yaw_deg=0.0)

        # 전역 헤딩 목표 초기화(현재 IMU yaw)
        _, _, yaw0 = self.imu_est.get_pose()
        self.yaw_target_deg = float(yaw0)

        # Goal 파싱
        try:
            steps = self._parse_route_json(goal_handle.request.route_json)
        except Exception as e:
            msg = f'route_json parse failed: {e}'
            self.get_logger().error(msg)
            goal_handle.abort()
            x, y, yaw = self.imu_est.get_pose()
            return FollowPath.Result(success=False, message=msg,
                                     total_time_s=0.0, final_x_m=x, final_y_m=y, final_yaw_deg=yaw)

        t_start = time.time()
        success = True

        for i, step in enumerate(steps):
            if goal_handle.is_cancel_requested:
                self._stop_joy()
                x, y, yaw = self.imu_est.get_pose()
                goal_handle.canceled()
                return FollowPath.Result(success=False, message='Canceled by client',
                                         total_time_s=time.time()-t_start, final_x_m=x, final_y_m=y, final_yaw_deg=yaw)

            if 'forward_m' in step:
                dist = float(step['forward_m'])
                lin_sign = 1.0 if dist >= 0.0 else -1.0
                self.get_logger().info(f'[{i}/{len(steps)-1}] forward {dist:.3f} m')

                # 기준점 저장
                x0, y0, _ = self.imu_est.get_pose()

                ok = self._run_forward(goal_handle, i,
                                       distance_m=abs(dist), lin_sign= -1* lin_sign, # 헤딩 반대 -1 곱합
                                       x0=x0, y0=y0)
                if not ok:
                    success = False
                    break

            elif 'turn_deg' in step:
                ang = -1 * float(step['turn_deg']) # 헤딩 반대 -1 곱함
                self.get_logger().info(f'[{i}/{len(steps)-1}] turn {ang:.1f} deg')

                # 전역 목표 헤딩 갱신만 수행(즉시 정확히 맞출 필요 없음 — 연이어 전진에서 보정)
                self.yaw_target_deg = self._wrap_deg(self.yaw_target_deg + ang)

                ok = self._run_turn(goal_handle, i)  # 2단계 제어로 "거의" 맞춘다
                if not ok:
                    success = False
                    break

            else:
                self.get_logger().warn(f'Step {i} has no forward_m/turn_deg. Skip.')

        self._stop_joy()
        x, y, yaw = self.imu_est.get_pose()
        msg = 'Route completed.' if success else 'Route aborted.'
        if success: goal_handle.succeed()
        else:       goal_handle.abort()

        return FollowPath.Result(success=success, message=msg,
                                 total_time_s=time.time()-t_start,
                                 final_x_m=x, final_y_m=y, final_yaw_deg=yaw)

    # --------------- 스텝 러너 --------------- #

    def _run_forward(self, goal_handle, step_index: int,
                 distance_m: float, lin_sign: float,
                 x0: float, y0: float) -> bool:
        """
        전진: 헤딩 P제어는 유지하고, 종결은 '거리/속도'로 얻은 시간(duration)만큼 동작.
        - 속도: self.x_vel [m/s]
        - 전진 축 크기: self.lin_axis_mag (조이스틱 스케일)
        """
        # 이동에 필요한 시간 계산 (안전 가드 포함)
        v = max(1e-6, float(self.x_vel))                  # [m/s]
        duration_s = abs(float(distance_m)) / v           # [s]
        t0 = time.time()
        t_end = t0 + duration_s

        joy = self._make_joy_msg()

        while True:
            if goal_handle.is_cancel_requested:
                self._stop_joy()
                return False

            now = time.time()
            if now >= t_end:
                # 목표 시간 도달 → 정지 및 성공
                self._stop_joy()
                return True

            # IMU로 현재 자세
            x, y, yaw_deg = self.imu_est.get_pose()

            # 헤딩 P 제어 (전진 중 yaw 보정)
            err = self._angle_diff_deg(self.yaw_target_deg, yaw_deg)
            yaw_rate_cmd = self.kp_yaw * err                    # [deg/s]
            yaw_axis = max(-1.0, min(1.0, yaw_rate_cmd / max(1e-6, self.yaw_rate_deg_s)))
            if self.min_yaw_axis > 0.0 and abs(yaw_axis) > 1e-3 and abs(yaw_axis) < self.min_yaw_axis:
                yaw_axis = math.copysign(self.min_yaw_axis, yaw_axis)

            # 전/후진 축: 조이스틱 스케일 파라미터 사용(속도는 duration에만 반영)
            joy.axes[self.idx_lin] = float(self.lin_axis_mag) * float(lin_sign)
            joy.axes[self.idx_yaw] = float(yaw_axis)
            self.joy_pub.publish(joy)

            # 피드백 (남은 시간 기반)
            fb = FollowPath.Feedback()
            fb.current_index = step_index
            fb.elapsed_time_s = float(now - t0)
            fb.remaining_time_s = max(0.0, float(t_end - now))
            fb.x_m = float(x); fb.y_m = float(y); fb.yaw_deg = float(yaw_deg)
            goal_handle.publish_feedback(fb)

            time.sleep(self.dt)


    def _run_turn(self, goal_handle, step_index: int) -> bool:
        """
        2단계 회전(고정 속도만 사용):
          - |err| > fast_window: turn_fast_axis로 회전
          - fast_window >= |err| > yaw_tol_deg: turn_slow_axis(절반 속도)로 회전
          - |err| <= yaw_tol_deg: 정지 후 종료 (잔오차는 전진에서 보정)
        턴 중 ZUPT로 드리프트 완화.
        """
        t0 = time.time()
        joy = self._make_joy_msg()

        while True:
            if goal_handle.is_cancel_requested:
                return False

            x, y, yaw_deg = self.imu_est.get_pose()
            err = -1*self._angle_diff_deg(self.yaw_target_deg, yaw_deg)

            # ZUPT
            self.imu_est.vx = 0.0
            self.imu_est.vy = 0.0

            ae = abs(err)
            if ae <= self.yaw_tol_deg:
                self._stop_joy()
                return True
            elif ae <= self.turn_fast_window_deg:
                yaw_axis = self.turn_slow_axis * (1.0 if err > 0 else -1.0)  # ★ 절반 고정 속도
            else:
                yaw_axis = self.turn_fast_axis * (1.0 if err > 0 else -1.0)  # 빠른 고정 속도

            joy.axes[self.idx_lin] = 0.0
            joy.axes[self.idx_yaw] = yaw_axis
            self.joy_pub.publish(joy)

            fb = FollowPath.Feedback()
            fb.current_index = step_index
            fb.elapsed_time_s = time.time() - t0
            fb.remaining_time_s = 0.0
            fb.x_m = float(x); fb.y_m = float(y); fb.yaw_deg = float(yaw_deg)
            goal_handle.publish_feedback(fb)

            if (time.time() - t0) > self.max_step_time_s:
                self.get_logger().warn('Turn step timeout.')
                return False

            time.sleep(self.dt)

    # ----------------- 유틸 ----------------- #

    def _on_imu(self, msg: Imu):
        self._imu_rx += 1
        self.imu_est.update(msg)

    def _imu_watchdog(self):
        self.get_logger().info(f'IMU rx ~ {self._imu_rx} msgs/s')
        self._imu_rx = 0

    def _wait_imu_ready(self, timeout_s: float) -> bool:
        t0 = time.time()
        while not self.imu_est.ready and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        return self.imu_est.ready

    def _parse_route_json(self, route_json: str) -> List[Dict[str, Any]]:
        steps = json.loads(route_json)
        if not isinstance(steps, list):
            raise ValueError('route_json must be a list.')
        out: List[Dict[str, Any]] = []
        for s in steps:
            if isinstance(s, dict) and (('forward_m' in s) ^ ('turn_deg' in s)):
                out.append(s)
        return out

    def _make_joy_msg(self) -> Joy:
        axes_len = max(self.idx_lin, self.idx_yaw) + 1
        joy = Joy()
        joy.axes = [0.0] * max(axes_len, 8)
        joy.buttons = [0] * 12
        return joy

    def _stop_joy(self):
        self.joy_pub.publish(self._make_joy_msg())

    @staticmethod
    def _wrap_deg(a: float) -> float:
        if a > 180.0: a -= 360.0
        if a < -180.0: a += 360.0
        return a

    @staticmethod
    def _angle_diff_deg(target: float, source: float) -> float:
        diff = target - source
        if diff > 180.0: diff -= 360.0
        if diff < -180.0: diff += 360.0
        return diff


# 맨 위에 추가
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()
    node = FollowPathServer()
    executor = MultiThreadedExecutor()     # ★ 멀티스레드 실행기
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
