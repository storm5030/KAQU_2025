from kaqu_controller.Kaquctrl.TrotGaitController import (TrotGaitController, TrotSwingController, TrotStanceController)
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
    if hasattr(command, "yaw_rate"):
        try:
            wz = float(command.yaw_rate)
        except Exception:
            pass
    return vx, vy, wz


class StairTrotGaitController(TrotGaitController):
    """
    계단 주행용 Trot 게이트 컨트롤러.
    - IMU 보정 완화
    - 앞다리/뒷다리 스윙 높이 개별 게인 (여기서는 '앞다리 게인' 중심으로 튜닝)
    """

    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        super().__init__(default_stance, stance_time, swing_time, time_step, use_imu)

        # 파라미터 로드
        leg = LegParameters()
        self.z_leg_lift = leg.stair.z_leg_lift
        self.max_x_vel = leg.stair.max_x_vel
        self.max_y_vel = leg.stair.max_y_vel
        self.max_yaw_rate = leg.stair.max_yaw_rate

        # z 에러 게인 (기존 설계 유지)
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
        self.pid_controller = PID_controller(0.12, 0.30, 0.01, 0.15, 0.05, 0.20, 0.10, 0.5)
        self.pid_controller.reset()

        # === stairmode에서 IMU 추종 완화 ===
        self.imu_follow_gain = 0.05
        self.imu_max_corr = 0.15
        self.imu_lpf_alpha = 0.2
        self._lp_roll_cmd = 0.0
        self._lp_pitch_cmd = 0.0

        # === 다리 인덱스 순서: FR, FL, RR, RL = [0, 1, 2, 3] ===
        # 앞다리(Front): FR, FL -> [0, 1]
        # 뒷다리(Rear):  RR, RL -> [2, 3]
        self.front_leg_indices = getattr(leg.stair, "front_leg_indices", [0, 1])
        self.rear_leg_indices  = getattr(leg.stair, "rear_leg_indices",  [2, 3])

        # 앞다리 스윙 높이 게인(요청사항: 앞다리 gain으로 조정)
        self.front_lift_gain = getattr(leg.stair, "front_lift_gain", 1.20)  # 필요에 따라 0.8~1.6 조정
        # 뒷다리 기본 1.0 (필요 시 따로 키울 수 있음)
        self.rear_lift_gain  = getattr(leg.stair, "rear_lift_gain", 1.40)

        # 스윙 중인데 접촉이 남아있으면 여유고도(미터)
        self.extra_clearance = getattr(leg.stair, "extra_clearance", 0.03)

        # SwingController에도 공유
        self.swingController.front_leg_indices = self.front_leg_indices
        self.swingController.rear_leg_indices  = self.rear_leg_indices
        self.swingController.front_lift_gain   = self.front_lift_gain
        self.swingController.rear_lift_gain    = self.rear_lift_gain
        self.swingController.extra_clearance   = self.extra_clearance

        # (선택) 트롯 contact 패턴을 안전한 대각 교대로 강제하고 싶다면 아래 활성화
        # self.contact_phases = np.array([
        #     [1, 0, 1, 0],  # FR
        #     [0, 1, 0, 1],  # FL
        #     [1, 0, 1, 0],  # RR
        #     [0, 1, 0, 1],  # RL
        # ], dtype=int)

    def _soft_clip(self, x, limit):
        if x > limit: return limit
        if x < -limit: return -limit
        return x

    def _lpf(self, prev, new, alpha):
        return alpha * prev + (1.0 - alpha) * new

    def run(self, state, command):
        """한 틱마다: trot step -> IMU 완화 보정 -> 발 위치 반환"""
        vx, vy, wz = _get_velocity3(command)
        command.velocity = [np.clip(vx, -self.max_x_vel,  self.max_x_vel),
                            np.clip(vy, -self.max_y_vel,  self.max_y_vel),
                            np.clip(wz, -self.max_yaw_rate, self.max_yaw_rate)]
        command.yaw_rate = command.velocity[2]

        # 기본 게이트 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height

        new_foot_locations = state.foot_location.copy()

        # === (A) IMU 보정(완화/부드럽게) ===
        if self.use_imu:
            roll = float(getattr(state, "imu_roll", 0.0))
            pitch = float(getattr(state, "imu_pitch", 0.0))
            inverted = bool(getattr(state, "is_inverted", False))

            if self.imu_gate and (abs(roll) > self.th_off or abs(pitch) > self.th_off or inverted):
                self.imu_gate = False
                self.pid_controller.reset()
                self._lp_roll_cmd = 0.0
                self._lp_pitch_cmd = 0.0
            elif (not self.imu_gate) and (abs(roll) < self.th_on and abs(pitch) < self.th_on and not inverted):
                self.imu_gate = True

            if self.imu_gate:
                roll_corr, pitch_corr = self.pid_controller.run(roll, pitch)
                roll_corr  = self._soft_clip(roll_corr,  self.imu_max_corr)
                pitch_corr = self._soft_clip(pitch_corr, self.imu_max_corr)

                roll_cmd  = self.imu_follow_gain * roll_corr
                pitch_cmd = self.imu_follow_gain * pitch_corr

                self._lp_roll_cmd  = self._lpf(self._lp_roll_cmd,  roll_cmd,  self.imu_lpf_alpha)
                self._lp_pitch_cmd = self._lpf(self._lp_pitch_cmd, pitch_cmd, self.imu_lpf_alpha)

                cr = np.cos(self._lp_roll_cmd)
                cp = np.cos(self._lp_pitch_cmd)
                tr = np.tan(self._lp_roll_cmd)
                tp = np.tan(self._lp_pitch_cmd)

                for leg_index in range(4):
                    is_stance = None
                    if hasattr(state, "contact"):
                        try:
                            is_stance = (int(state.contact[leg_index]) == 1)
                        except Exception:
                            is_stance = None

                    # z 보정: stance 위주
                    dz = 0.0
                    if is_stance is None or is_stance:
                        new_z = command.robot_height * cp * cr
                        dz = -1.0 * (command.robot_height - new_z)

                    new_foot_locations[2, leg_index] += dz

                    # x,y 보정: swing에는 약하게(0.3배)
                    xy_gain = 1.0 if (is_stance is None or is_stance) else 0.3
                    z_now_after = new_foot_locations[2, leg_index]
                    dx = (-1.0 * z_now_after) * tp * xy_gain
                    dy = ( 1.0 * z_now_after) * tr * xy_gain

                    new_foot_locations[0, leg_index] += dx
                    new_foot_locations[1, leg_index] += dy

        return new_foot_locations


class StairSwingController(TrotSwingController):
    """
    계단 스윙 단계:
    - 앞/뒤 다리 스윙 높이 게인 분리(여기서는 '앞다리 게인' 중심)
    - 스윙 중 접촉이면 extra_clearance 추가
    """
    # 기본값(컨트롤러에서 override)
    front_leg_indices = [0, 1]  # FR, FL
    rear_leg_indices  = [2, 3]  # RR, RL
    front_lift_gain = 1.20
    rear_lift_gain  = 1.00
    extra_clearance = 0.03  # m

    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        vx, vy, wz = _get_velocity3(command)

        if swing_phase < 0.5:
            return self.default_stance[:, leg_index]
        else:
            delta_pos_2d = 2 * np.array([vx, vy]) * self.phase_length * self.time_step
            delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0.0])

            theta = self.stance_ticks * self.time_step * wz * 2.0
            rotation = rotz(theta)
            return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def swing_height(self, swing_phase, leg_index=None, in_contact=False):
        # 기본 삼각 프로파일
        if swing_phase < 0.5:
            swing_h = (swing_phase / 0.5) * self.z_leg_lift
        else:
            swing_h = self.z_leg_lift * (1.0 - (swing_phase - 0.5) / 0.5)

        # 앞/뒤 다리 별 스윙 높이 게인 적용
        if leg_index is not None:
            if leg_index in self.front_leg_indices:
                swing_h *= self.front_lift_gain
            elif leg_index in self.rear_leg_indices:
                swing_h *= self.rear_lift_gain

        # 스윙 중인데 접촉이 남아있으면 여유고도 추가
        if in_contact:
            swing_h += self.extra_clearance

        return swing_h

    def next_foot_location(self, swing_prop, leg_index, state, command):
        # 진행률 한 틱만큼 전진(0~0.9 -> 0.1~1.0)
        swing_prop += (1.0 / self.swing_ticks)

        foot_location = state.foot_location[:, leg_index]
        in_contact = False
        if hasattr(state, "contact"):
            try:
                in_contact = bool(int(state.contact[leg_index]) == 1)
            except Exception:
                in_contact = False

        swing_h = self.swing_height(swing_prop, leg_index=leg_index, in_contact=in_contact)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, swing_prop)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)

        # 마지막 틱: 착지 위치로 정렬
        if swing_prop >= 1.0:
            new_position = touchdown_location * np.array([1.0, 1.0, 0.0]) + np.array([0.0, 0.0, command.robot_height])
            return new_position  # [x, y, z]

        # XY는 터치다운으로 선형 보간
        velocity_xy = (touchdown_location - foot_location) / float(max(time_left, 1e-6)) * np.array([1.0, 1.0, 0.0])
        delta_xy = velocity_xy * self.time_step

        # Z는 swing_height + 로봇 목표 높이
        z_vec = np.array([0.0, 0.0, swing_h + command.robot_height])

        return foot_location * np.array([1.0, 1.0, 0.0]) + z_vec + delta_xy
