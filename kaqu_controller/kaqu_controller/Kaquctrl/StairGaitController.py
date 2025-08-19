# -*- coding: utf-8 -*-
# StairGaitController.py

from kaqu_controller.Kaquctrl.TrotGaitController import (
    TrotGaitController, TrotSwingController, TrotStanceController
)
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters
from kaqu_controller.KaquIK.KinematicsCalculations import rotz
import numpy as np


def _get_velocity3(command):
    """command.velocity를 (vx, vy, wz)로 안전하게 반환하고, yaw_rate가 없으면 0.0으로 보정"""
    vx, vy, wz = 0.0, 0.0, 0.0
    if hasattr(command, "velocity") and command.velocity is not None:
        try:
            if len(command.velocity) >= 2:
                vx, vy = float(command.velocity[0]), float(command.velocity[1])
            if len(command.velocity) >= 3:
                wz = float(command.velocity[2])
        except Exception:
            pass
    # 우선순위: command.yaw_rate가 있으면 wz로 사용
    if hasattr(command, "yaw_rate"):
        try:
            wz = float(command.yaw_rate)
        except Exception:
            pass
    return vx, vy, wz


class StairTrotGaitController(TrotGaitController):
    """
    계단 주행용 Trot 게이트 컨트롤러(파이썬).
    - IMU 보정량 스케일/제한/로우패스 적용으로 몸통이 경사를 '덜' 따르도록 함
    - 기존 인터페이스 최대한 유지
    """

    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        super().__init__(default_stance, stance_time, swing_time, time_step, use_imu)

        # 파라미터 로드
        leg = LegParameters()
        self.z_leg_lift = leg.stair.z_leg_lift
        self.max_x_vel = leg.stair.max_x_vel
        self.max_y_vel = leg.stair.max_y_vel
        self.max_yaw_rate = leg.stair.max_yaw_rate

        # z 에러 게인 (기존 설계 따라 유지)
        z_error_constant = 0.5 * 4

        # 컨트롤러 구성
        self.swingController = StairSwingController(
            self.stance_ticks,
            self.swing_ticks,
            self.time_step,
            self.phase_length,
            self.z_leg_lift,
            self.default_stance,
        )

        self.stanceController = TrotStanceController(
            self.phase_length,
            self.stance_ticks,
            self.swing_ticks,
            self.time_step,
            z_error_constant,
        )

        # PID 컨트롤러
        # (kp, ki, kd, meas_tau, d_tau, out_limit, i_limit, deadband_deg)
        self.pid_controller = PID_controller(0.12, 0.30, 0.01, 0.15, 0.05, 0.20, 0.10, 0.5)
        self.pid_controller.reset()

        # === stairmode에서 IMU 추종 완화용 파라미터 ===
        # 0.0=수평 유지, 1.0=IMU 그대로 따름. 0.25~0.5 권장.
        self.imu_follow_gain = 0.10
        # 보정 각 최대치(라디안 기준: 0.15rad ≈ 8.6도). 구현 단위가 '도'라면 8.0 정도로 맞춰주세요.
        self.imu_max_corr = 0.15
        # 1차 로우패스 비율(작을수록 더 천천히 추종)
        self.imu_lpf_alpha = 0.2

        # 로우패스 내부 상태
        self._lp_roll_cmd = 0.0
        self._lp_pitch_cmd = 0.0

    # ----------------- 유틸리티 -----------------
    def _soft_clip(self, x, limit):
        if x > limit:
            return limit
        if x < -limit:
            return -limit
        return x

    def _lpf(self, prev, new, alpha):
        return alpha * prev + (1.0 - alpha) * new

    # ----------------- 메인 루프 -----------------
    def run(self, state, command):
        """
        외부에서 한 틱마다 호출.
        1) 베이스 trot step 실행
        2) stairmode 전용 IMU 보정(경사 추종 완화)
        3) 발 위치 반환
        """
        # 방어: 속도 명령 보정(2D만 들어오면 wz=0.0으로 확장)
        vx, vy, wz = _get_velocity3(command)
        command.velocity = [vx, vy, wz]
        command.yaw_rate = wz

        # 속도 클램프 (LegParameters 기반)
        vx = np.clip(vx, -self.max_x_vel, self.max_x_vel)
        vy = np.clip(vy, -self.max_y_vel, self.max_y_vel)
        wz = np.clip(wz, -self.max_yaw_rate, self.max_yaw_rate)
        command.velocity = [vx, vy, wz]
        command.yaw_rate = wz

        # 기본 게이트 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height

        new_foot_locations = state.foot_location.copy()

        # === (A) IMU 보정: stairmode에서는 경사 추종을 약하게/부드럽게 ===
        if self.use_imu:
            roll = float(getattr(state, "imu_roll", 0.0))
            pitch = float(getattr(state, "imu_pitch", 0.0))
            inverted = bool(getattr(state, "is_inverted", False))

            # 게이트 온/오프 히스테리시스
            if self.imu_gate and (abs(roll) > self.th_off or abs(pitch) > self.th_off or inverted):
                self.imu_gate = False
                self.pid_controller.reset()
                self._lp_roll_cmd = 0.0
                self._lp_pitch_cmd = 0.0
            elif (not self.imu_gate) and (abs(roll) < self.th_on and abs(pitch) < self.th_on and not inverted):
                self.imu_gate = True

            if self.imu_gate:
                # PID 출력(단위는 기존 구현을 따름)
                roll_corr, pitch_corr = self.pid_controller.run(roll, pitch)

                # 1) 최대 보정 각 제한
                roll_corr = self._soft_clip(roll_corr, self.imu_max_corr)
                pitch_corr = self._soft_clip(pitch_corr, self.imu_max_corr)

                # 2) 경사 추종 게인으로 축소
                roll_cmd = self.imu_follow_gain * roll_corr
                pitch_cmd = self.imu_follow_gain * pitch_corr

                # 3) 로우패스(부드럽게)
                self._lp_roll_cmd = self._lpf(self._lp_roll_cmd, roll_cmd, self.imu_lpf_alpha)
                self._lp_pitch_cmd = self._lpf(self._lp_pitch_cmd, pitch_cmd, self.imu_lpf_alpha)

                # 사전 계산
                cr = np.cos(self._lp_roll_cmd)
                cp = np.cos(self._lp_pitch_cmd)
                tr = np.tan(self._lp_roll_cmd)
                tp = np.tan(self._lp_pitch_cmd)

                for leg_index in range(4):
                    # stance/swing 판단이 가능하면 stance에 더 강하게 적용
                    is_stance = None
                    if hasattr(state, "contact"):
                        try:
                            is_stance = (int(state.contact[leg_index]) == 1)
                        except Exception:
                            is_stance = None

                    # z 보정: stance 위주
                    if is_stance is None or is_stance:
                        new_z = command.robot_height * cp * cr
                        dz = -1.0 * (command.robot_height - new_z)
                    else:
                        dz = 0.0

                    new_foot_locations[2, leg_index] += dz

                    # x,y 보정: swing에는 약하게(0.3배)
                    xy_gain = 1.0 if (is_stance is None or is_stance) else 0.3
                    z_now_after = new_foot_locations[2, leg_index]
                    dx = (-1.0 * z_now_after) * tp * xy_gain
                    dy = (1.0 * z_now_after) * tr * xy_gain

                    new_foot_locations[0, leg_index] += dx
                    new_foot_locations[1, leg_index] += dy

        return new_foot_locations


class StairSwingController(TrotSwingController):
    """
    계단 스윙 단계:
    - 초반(0~0.5): 들어올리기
    - 후반(0.5~1.0): 내려놓기
    - Raibert 터치다운 위치에 yaw 회전/전진 보간 적용
    """

    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        # 안전한 (vx, vy, wz) 추출
        vx, vy, wz = _get_velocity3(command)

        if swing_phase < 0.5:
            return self.default_stance[:, leg_index]
        else:
            # 2 * v * T_half  (기본 설계 유지)
            delta_pos_2d = 2 * np.array([vx, vy]) * self.phase_length * self.time_step
            delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0.0])

            theta = self.stance_ticks * self.time_step * wz * 2.0
            rotation = rotz(theta)

            return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def swing_height(self, swing_phase):
        # 0.5까지 선형 상승, 이후 선형 하강
        if swing_phase < 0.5:
            swing_height_ = (swing_phase / 0.5) * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1.0 - (swing_phase - 0.5) / 0.5)
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command):
        """
        스윙 진행률을 한 틱 진전시키고, 발끝 목표를 반환
        - 마지막 틱에는 touchdown 위치를 정확히 반환
        """
        # 진행률 한 틱만큼 전진(0~0.9 -> 0.1~1.0)
        swing_prop += (1.0 / self.swing_ticks)

        foot_location = state.foot_location[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, swing_prop)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)

        # 마지막 틱: 착지 위치로 정렬
        if swing_prop >= 1.0:
            new_position = touchdown_location * np.array([1.0, 1.0, 0.0]) + np.array([0.0, 0.0, command.robot_height])
            return new_position  # [x, y, z]

        # XY는 터치다운으로 선형 보간 속도
        velocity_xy = (touchdown_location - foot_location) / float(max(time_left, 1e-6)) * np.array([1.0, 1.0, 0.0])
        delta_xy = velocity_xy * self.time_step

        # Z는 swing_height + 로봇 목표 높이
        z_vec = np.array([0.0, 0.0, swing_height_ + command.robot_height])

        return foot_location * np.array([1.0, 1.0, 0.0]) + z_vec + delta_xy
