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

                # --- 계단 규격 반영 (stair사용) ---
        self.stair_tread = 0.20   # 계단 폭(앞으로 가야하는 길이)
        self.stair_rise  = 0.03   # 계단 높이
        # 안전 여유
        self.step_margin = 0.02   # 스텝 길이 여유
        self.lift_margin = 0.03   # 스윙 높이 여유

        # 최소 보장값
        self.min_step_length = self.stair_tread + self.step_margin   # 0.12 m
        self.min_swing_lift  = self.stair_rise  + self.lift_margin   # 0.08 m

        # z 에러 게인 (기존 설계 유지)
        z_error_constant = 0.5 * 4

        # 컨트롤러 구성
        self.swingController = StairSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
            self.phase_length, self.z_leg_lift, self.default_stance)

        self.stanceController = TrotStanceController(
            self.phase_length, self.stance_ticks,self.swing_ticks,
            self.time_step, z_error_constant)

        # PID 컨트롤러
        self.pid_controller = PID_controller(0.12, 0.30, 0.01, 0.15, 0.05, 0.20, 0.10, 0.5)
        self.pid_controller.reset()

        # === stairmode에서 IMU 추종 완화 ===
        self.imu_follow_gain = 0.12
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
        self.rear_lift_gain  = getattr(leg.stair, "rear_lift_gain", 1.50)

        # 스윙 중인데 접촉이 남아있으면 여유고도(미터)
        self.extra_clearance = getattr(leg.stair, "extra_clearance", 0.03)

        # SwingController에 전달 (최소 보장값 포함!)
        self.swingController.front_leg_indices = self.front_leg_indices
        self.swingController.rear_leg_indices  = self.rear_leg_indices
        self.swingController.front_lift_gain   = self.front_lift_gain
        self.swingController.rear_lift_gain    = self.rear_lift_gain
        self.swingController.extra_clearance   = self.extra_clearance
        self.swingController.min_step_length   = self.min_step_length
        self.swingController.min_swing_lift    = self.min_swing_lift
 
        # --- FF(선행) 피치 보정 파라미터 ---
        self.ff_pitch_deg = 12.0      # 최대 앞숙임 각도(도). 6~12도 사이에서 튜닝
        self.ff_ramp_start = 0.15    # 스윙 진행률 20%부터 보정 시작
        self.ff_ramp_end   = 0.95    # 스윙 진행률 80%에 최대치 도달
        self.ff_height_drop = 0.015   # rear 스윙 중 전체 COM 약간 낮추기(미터, 0~1cm 권장)



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

        # === (B) rear 스윙시에만 FF(선행) 피치로 전방 숙이기 + COM 살짝 낮추기 ===
        # 현재 phase 접지 패턴: [FR, FL, RR, RL] = [0,1,2,3]
        contact_modes = self.contacts(state.ticks)
        rear_swinging = (contact_modes[2] == 0) or (contact_modes[3] == 0)

        if rear_swinging:
            # 현재 phase 안에서 스윙 진행률(0~1)
            swing_prop = float(self.subphase_ticks(state.ticks)) / float(max(self.swing_ticks, 1))

            # 램프 계수: ff_ramp_start~ff_ramp_end 구간에서 0 -> 1
            s0, s1 = self.ff_ramp_start, self.ff_ramp_end
            if swing_prop <= s0:
                r = 0.0
            elif swing_prop >= s1:
                r = 1.0
            else:
                r = (swing_prop - s0) / max((s1 - s0), 1e-6)

            # 목표 피치(라디안): "앞으로 숙이기" => 음수 아니면 P_ff를 +로 변경
            p_ff = -np.deg2rad(self.ff_pitch_deg) * r
            t_pitch = np.tan(p_ff)

            # z 보정: z += x * tan(pitch)
            # (x>0가 앞다리, x<0가 뒷다리라는 좌표계를 가정)
            for i in range(4):
                x = new_foot_locations[0, i]
                new_foot_locations[2, i] += x * t_pitch

            # COM 살짝 낮추기(과하면 충돌/미끄럼 가능, 0~1cm 권장)
            dz_drop = self.ff_height_drop * r
            new_foot_locations[2, :] -= dz_drop

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
    front_lift_gain = 1.00
    rear_lift_gain  = 1.20
    extra_clearance = 0.03  # m

    # 새로 전달받는 보장값(stairs)
    min_step_length = 0.12  # m
    min_swing_lift  = 0.08  # m


    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        vx, vy, wz = _get_velocity3(command)

        if swing_phase < 0.5:
            # 초반은 제자리 근처에서 들어올리기
            return self.default_stance[:, leg_index]
        else:
            # 기본: 2 * v * T_half
            base_delta_2d = 2 * np.array([vx, vy]) * self.phase_length * self.time_step

            # 최소 보폭(0.12m)을 스윙 후반으로 갈수록 강하게 적용
            # 0.5 -> 0.0, 1.0 -> 1.0 로 선형 램프
            lam = (swing_phase - 0.5) / 0.5
            lam = max(0.0, min(1.0, lam))

            # 목표 step_x: base와 최소보폭 사이 보간 (방향은 vx 부호, 거의 0이면 +방향 가정)
            step_x_min = self.min_step_length
            sign = np.sign(vx) if abs(vx) > 1e-3 else 1.0
            step_x_target = (1.0 - lam) * base_delta_2d[0] + lam * (sign * step_x_min)

            delta_pos = np.array([step_x_target, base_delta_2d[1], 0.0])

            theta = self.stance_ticks * self.time_step * wz * 2.0
            rotation = rotz(theta)
            return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

        
    rear_lift_delay = 0.35  # 이 구간 전에는 증폭 X
    rear_lift_full  = 0.70  # 이 이후부터 full rear_lift_gain


    def swing_height(self, swing_phase, leg_index=None, in_contact=False):
        # 기본 삼각 프로파일
        if swing_phase < 0.5:
            swing_h = (swing_phase / 0.5) * self.z_leg_lift
        else:
            swing_h = self.z_leg_lift * (1.0 - (swing_phase - 0.5) / 0.5)

        # 앞/뒤 다리 별 게인 (앞다리는 즉시, 뒷다리는 "지연 + 램프")
        if leg_index is not None:
            if leg_index in self.front_leg_indices:
                swing_h *= self.front_lift_gain
            elif leg_index in self.rear_leg_indices:
                # rear: swing_phase < delay => 1.0, delay~full 사이 선형 램프, 이후 full
                if swing_phase <= self.rear_lift_delay:
                    gain = 1.0
                elif swing_phase >= self.rear_lift_full:
                    gain = self.rear_lift_gain
                else:
                    # 선형 램프
                    t = ((swing_phase - self.rear_lift_delay) /
                         max(self.rear_lift_full - self.rear_lift_delay, 1e-6))
                    gain = 1.0 + t * (self.rear_lift_gain - 1.0)
                swing_h *= gain

        # 스윙 중인데 접촉이 남아있으면 여유고도 추가
        if in_contact:
            swing_h += self.extra_clearance

        # --- 최소 스윙 높이 강제 (계단 높이+여유) ---
        swing_h = max(swing_h, self.min_swing_lift)
        return swing_h

    def next_foot_location(self, swing_prop, leg_index, state, command):
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

        if swing_prop >= 1.0:
            new_position = touchdown_location * np.array([1.0, 1.0, 0.0]) + np.array([0.0, 0.0, command.robot_height])
            return new_position

        velocity_xy = (touchdown_location - foot_location) / float(max(time_left, 1e-6)) * np.array([1.0, 1.0, 0.0])
        delta_xy = velocity_xy * self.time_step
        z_vec = np.array([0.0, 0.0, swing_h + command.robot_height])

        return foot_location * np.array([1.0, 1.0, 0.0]) + z_vec + delta_xy