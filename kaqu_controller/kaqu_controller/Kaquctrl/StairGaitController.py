# === Minimal Stair Controller (A + IMU PID/LPF correction only, mm-consistent) ===
from kaqu_controller.Kaquctrl.TrotGaitController import (TrotGaitController, TrotSwingController, TrotStanceController)
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters
from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz, rotz
import numpy as np


def _get_velocity3(command):
    """command.velocity를 (vx, vy, wz)로 안전하게 반환"""
    vx, vy, wz = 0.0, 0.0, 0.0
    if hasattr(command, "velocity") and command.velocity is not None:
        try:
            if len(command.velocity) >= 2:
                vx, vy = float(command.velocity[0]), float(command.velocity[1])
            if len(command.velocity) >= 3:
                wz = float(command.velocity[2])
        except Exception:
            pass
    if hasattr(command, "yaw_rate"):
        try:
            wz = float(command.yaw_rate)
        except Exception:
            pass
    return vx, vy, wz


class StairTrotGaitController(TrotGaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        super().__init__(default_stance, stance_time, swing_time, time_step, use_imu)

        leg = LegParameters()
        self.z_leg_lift = leg.stair.z_leg_lift        # mm
        self.max_x_vel  = leg.stair.max_x_vel         # m/s
        self.max_y_vel  = leg.stair.max_y_vel         # m/s 
        self.max_yaw_rate = leg.stair.max_yaw_rate    # rad/s

        # z 에러 게인(기존 설계 값 범위 유지)
        z_error_constant = 1.0

        self.swingController = StairSwingController(
            self.stance_ticks, self.swing_ticks, self.time_step,
            self.phase_length, self.z_leg_lift, self.default_stance
        )

        self.stanceController = TrotStanceController(
            self.phase_length, self.stance_ticks, self.swing_ticks,
            self.time_step, z_error_constant
        )

        # === IMU PID/LPF (B 스타일, 최소 파라미터만) ===
        # PID는 roll, pitch → 작은 보정값(무차원)으로 사용
        # 원하는 보정 세기는 imu_follow_gain으로 조정
        self.pid_controller = PID_controller(0.12, 0.30, 0.01)  # (kp, ki, kd) 단순 3항
        self.pid_controller.reset()

        self.imu_follow_gain = 0.05          # PID 결과에 곱하는 추종 게인
        self.imu_max_corr    = 0.08          # 소프트클립 최대치(라디안 수준의 작은 값)
        self.imu_lpf_alpha   = 0.4           # LPF alpha (0~1, 클수록 느림)
        self._lp_roll_cmd    = 0.0
        self._lp_pitch_cmd   = 0.0

        # 🔥 추가: FF 전방 숙임 파라미터
        self.ff_pitch_deg = 12.0   # 목표 전방 숙임 (deg)
        self.ff_gain = 1.0         # 크기 스케일
        self.ff_start = 0.2        # swing phase 시작점
        self.ff_end   = 0.9        # swing phase 끝점

    # --- 간단 contact 판단: state.contact가 있으면 쓰고, 없으면 stance로 간주
    def _get_in_contact(self, state, leg_index):
        if hasattr(state, "contact"):
            try:
                v = int(state.contact[leg_index])
                if v in (0, 1):
                    return bool(v)
            except Exception:
                pass
        return True  # 센서 없으면 보수적으로 stance로 취급

    def _soft_clip(self, x, limit):
        if x > limit: return limit
        if x < -limit: return -limit
        return x

    def _lpf(self, prev, new, alpha):
        # alpha는 '이전값 가중치' 스타일 (B 코드와 동일)
        return alpha * prev + (1.0 - alpha) * new

    def run(self, state, command):
        """
        1) 기본 trot 궤적 생성
        2) (옵션) IMU 보정: PID → 클립 → LPF → 발 위치에 소규모 보정
        """
        # 속도 제한(명목상 범위)
        vx, vy, wz = _get_velocity3(command)
        command.velocity = [
            np.clip(vx, -self.max_x_vel, self.max_x_vel),
            np.clip(vy, -self.max_y_vel, self.max_y_vel),
            np.clip(wz, -self.max_yaw_rate, self.max_yaw_rate),
        ]
        command.yaw_rate = command.velocity[2]

        # 1) 기본 보행 궤적 (base_link 좌표계, mm)
        state.foot_location = self.step(state, command)    # (3,4), mm
        state.robot_height  = command.robot_height         # mm
        new_foot_locations  = state.foot_location.copy()

        # 2) IMU 보정(PID+LPF)
        if self.use_imu:
            # IMU raw (단위는 rad 기준 가정. deg가 들어오면 상위에서 변환 필요)
            roll = float(getattr(state, "imu_roll", 0.0))
            pitch = float(getattr(state, "imu_pitch", 0.0))

            # PID 결과(작은 수치), 과도한 튐 방지: 소프트클립
            roll_corr, pitch_corr = self.pid_controller.run(roll, pitch)
            roll_corr  = self._soft_clip(roll_corr,  self.imu_max_corr)
            pitch_corr = self._soft_clip(pitch_corr, self.imu_max_corr)

            # 추종 게인
            roll_cmd  = self.imu_follow_gain * roll_corr
            pitch_cmd = self.imu_follow_gain * pitch_corr

            # LPF
            self._lp_roll_cmd  = self._lpf(self._lp_roll_cmd,  roll_cmd,  self.imu_lpf_alpha)
            self._lp_pitch_cmd = self._lpf(self._lp_pitch_cmd, pitch_cmd, self.imu_lpf_alpha)

            # 보정 치수 → 발 위치 반영
            # z 보정: stance 위주, xy 보정: swing에 약하게
            cr = np.cos(self._lp_roll_cmd)
            cp = np.cos(self._lp_pitch_cmd)
            tr = np.tan(self._lp_roll_cmd)
            tp = np.tan(self._lp_pitch_cmd)

            for leg_index in range(4):
                is_stance = self._get_in_contact(state, leg_index)

                # z 보정(stance): 바디 기울기에 따른 목표 높이 변화량
                dz = 0.0
                if is_stance:
                    new_z = state.robot_height * cp * cr
                    dz = -1.0 * (state.robot_height - new_z)   # mm

                new_foot_locations[2, leg_index] += dz

                # xy 보정: stance 1.0배, swing 0.3배 정도로 약하게
                xy_gain = 1.0 if is_stance else 0.3
                z_now = new_foot_locations[2, leg_index]
                dx = (-1.0 * z_now) * tp * xy_gain     # mm
                dy = ( 1.0 * z_now) * tr * xy_gain     # mm

                new_foot_locations[0, leg_index] += dx
                new_foot_locations[1, leg_index] += dy

        swing_prop = float(self.subphase_ticks(state.ticks)) / float(max(self.swing_ticks, 1))
        if self.ff_start < swing_prop < self.ff_end:
            # 목표 pitch (라디안)
            p_ff = np.deg2rad(self.ff_pitch_deg) * self.ff_gain
            t_pitch = np.tan(p_ff)

            # 앞다리 쪽만 z 낮춤 (FR=0, FL=1)
            for i in [0, 1]:
                x = new_foot_locations[0, i]
                new_foot_locations[2, i] += x * t_pitch   # x에 비례해 앞으로 고개 숙임
        
        return new_foot_locations

class StairSwingController(TrotSwingController):
    """
       단순 스윙 프로파일 유지.
    - 보폭 계산시 속도(m/s)를 mm/s로 변환해 일관성 확보.
    """
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)
        self.T_lift = 0.35
        self.T_forward = 0.7
        self.T_fall = 1.0

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        # 0) 초반엔 기본자세 유지
        if swing_phase < 0.5:
            return self.default_stance[:, leg_index]

        # 1) 속도(m/s) 안전 추출
        vx, vy, wz = _get_velocity3(command)  # m/s, m/s, rad/s

        # 2) 반주기 시간(s) * 속도 = 변위(m)
        dt_half = max(self.phase_length * self.time_step, 1e-6)  # s
        delta_m = np.array([vx, vy]) * dt_half                   # [m, m]

        # 3) default_stance의 단위 감지 (간단 휴리스틱)
        #   - 보통 m이면 수 cm~수십 cm = 0.x 수준, mm면 수십~수백 = 10~300 수준
        stance_vec = self.default_stance[:, leg_index]
        stance_mag_xy = float(np.linalg.norm(stance_vec[:2]))
        use_mm = (stance_mag_xy > 5.0)   # 5보다 크면 mm로 가정
        scale = 1000.0 if use_mm else 1.0

        # 4) 변위 단위를 default_stance에 맞춤
        delta = np.array([delta_m[0]*scale, delta_m[1]*scale, 0.0])

        # 5) 과보폭 방지(클램프): IK가 비도달 되지 않도록 1틱 보폭 상한
        #    값은 mm 기준으로 주고, m 좌표면 그대로 비례(= scale=1)
        max_step_xy = getattr(self, "max_raibert_step_xy", 80.0 if use_mm else 0.08)  # 80mm or 0.08m
        dxy = delta[:2]
        n = float(np.linalg.norm(dxy))
        if n > max_step_xy:
            delta[:2] = dxy * (max_step_xy / max(n, 1e-9))

        # 6) 요 회전 적용(라디안)
        theta = self.stance_ticks * self.time_step * wz * 2.0
        R = rotz(theta)  # 3x3

        # 7) 회전된 기본자세 + 보폭
        return (R @ stance_vec) + delta
    

    def swing_height(self, swing_phase):
        if swing_phase < self.T_lift:
            swing_height_ = swing_phase / self.T_lift * self.z_leg_lift
        elif swing_phase < self.T_forward:
            swing_height_ = self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * ((1.0 - swing_phase) / (1.0 - self.T_forward))
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command):
        swing_prop += (1.0 / self.swing_ticks)
        foot_location = state.foot_location[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, swing_prop)

        if swing_prop >= 1.0:
            # z는 로봇 기준 높이로 딱 맞춤
            new_position = touchdown_location * np.array([1.0, 1.0, 0.0]) + np.array([0.0, 0.0, command.robot_height])
            return new_position

        # lift/forward/fall 구간에 따른 평이한 XY 프로파일
        if swing_prop < self.T_lift:
            velocity = np.array([0.0, 0.0, 0.0])
        elif swing_prop < self.T_forward:
            time_left = self.time_step * self.swing_ticks * (self.T_forward - swing_prop)
            velocity = (touchdown_location - foot_location) / float(max(time_left, 1e-6)) * np.array([1.0, 1.0, 0.0])
        else:
            velocity = np.array([0.0, 0.0, 0.0])

        delta_foot_location = velocity * self.time_step
        z_vector = np.array([0.0, 0.0, swing_height_ + command.robot_height])
        return foot_location * np.array([1.0, 1.0, 0.0]) + z_vector + delta_foot_location
