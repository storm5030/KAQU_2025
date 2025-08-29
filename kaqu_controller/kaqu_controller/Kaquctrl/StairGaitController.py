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
        self.stair_tread = 100   # 계단 폭(앞으로 가야하는 길이)
        self.stair_rise  = 50   # 계단 높이
        # 안전 여유
        self.step_margin = 20
        self.lift_margin = 30

        # 최소 보장값
        self.min_step_length = self.stair_tread + self.step_margin    #실제 값은 계단 설정에 맞춤
        self.min_swing_lift  = 16   # 계단 기본 스윙 높이 여유 ↑(발 긁힘 방지)

        # z 에러 게인 (기존 설계 유지)
        z_error_constant = 0.5 * 2

        # 컨트롤러 구성
        self.swingController = StairSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
            self.phase_length, self.z_leg_lift, self.default_stance)

        self.stanceController = TrotStanceController(
            self.phase_length, self.stance_ticks, self.swing_ticks,
            self.time_step, z_error_constant)

        # PID 컨트롤러
        self.pid_controller = PID_controller(0.12, 0.30, 0.01, 0.15, 0.05, 0.20, 0.10, 0.5)
        self.pid_controller.reset()

        # === stairmode에서 IMU 추종 완화 ===
        self.imu_follow_gain = 0.05
        self.imu_max_corr    = 0.08
        self.imu_lpf_alpha   = 0.4
        self._lp_roll_cmd = 0.0
        self._lp_pitch_cmd = 0.0

        # === 다리 인덱스 순서: FR, FL, RR, RL = [0, 1, 2, 3] ===
        self.front_leg_indices = getattr(leg.stair, "front_leg_indices", [0, 1])
        self.rear_leg_indices  = getattr(leg.stair, "rear_leg_indices",  [2, 3])

        # 스윙 높이 게인
        self.front_lift_gain = getattr(leg.stair, "front_lift_gain", 1.2)
        self.rear_lift_gain  = getattr(leg.stair, "rear_lift_gain", 1.2)

        # 스윙 중 접촉 시 여유고도
        self.extra_clearance = getattr(leg.stair, "extra_clearance", 8)

        # SwingController에 전달 (최소 보장값 포함!)
        self.swingController.front_leg_indices = self.front_leg_indices
        self.swingController.rear_leg_indices  = self.rear_leg_indices
        self.swingController.front_lift_gain   = self.front_lift_gain
        self.swingController.rear_lift_gain    = self.rear_lift_gain
        self.swingController.extra_clearance   = self.extra_clearance
        self.swingController.min_step_length   = self.min_step_length
        self.swingController.min_swing_lift    = self.min_swing_lift

        # --- FF(선행) 피치 보정 파라미터 ---
        self.ff_pitch_deg = 10.0
        self.ff_ramp_start = 0.15
        self.ff_ramp_end   = 0.90
        self.ff_height_drop = 15
        self.ff_pitch_boost_when_single_rear = 1.00
        self.ff_drop_boost_when_single_rear  = 1.00

        # === (추가) 접촉 휴리스틱 파라미터 ===
        self.use_contact_heuristic = True
        self.heuristic_z_soft = 40   # 바디 기준 하한(예: -20cm)
        self.heuristic_z_margin = 10 # 여유 1cm

        # SwingController에도 공유
        self.swingController.use_contact_heuristic = self.use_contact_heuristic
        self.swingController.heuristic_z_soft = self.heuristic_z_soft
        self.swingController.heuristic_z_margin = self.heuristic_z_margin

        # <<< safety: 넘어짐/랩핑 failsafe 파라미터
        self.tip_roll_deg   = 35.0   # 이 이상 기울면 안전모드
        self.tip_pitch_deg  = 30.0
        self.hard_tip_deg   = 75.0   # 심각 기울기(전복 의심)
        self.safemode_hold_sec = 0.6
        self._safe_mode_until_tick = -1

    # --- contact 추정 (그대로)
    def _get_in_contact(self, state, leg_index, robot_height):
        if hasattr(state, "contact"):
            try:
                v = int(state.contact[leg_index])
                if v in (0, 1):
                    return bool(v)
            except Exception:
                pass
        if getattr(self, "use_contact_heuristic", False):
            try:
                z_soft = robot_height - float(getattr(self, "heuristic_z_soft", 20))
                margin = float(getattr(self, "heuristic_z_margin", 10))
                foot_z = float(state.foot_location[2, leg_index])  # 바디 좌표계 z
                return foot_z < (z_soft + margin)
            except Exception:
                return False
        return False

    def _soft_clip(self, x, limit):
        if x > limit: return limit
        if x < -limit: return -limit
        return x

    def _lpf(self, prev, new, alpha):
        return alpha * prev + (1.0 - alpha) * new

    # <<< safety: 각도 유틸
    def _normalize_angles_and_units(self, roll, pitch):
        """IMU가 도()로 들어오면 rad로 변환하고 [-π, π]로 랩핑."""
        # 단위 추정: 360° 수준이면 
        if abs(roll) > 3.0 or abs(pitch) > 3.0:
            # 값이 3 rad(≈172°)를 넘으면 deg일 확률 ↑
            if abs(roll) > np.pi*2 or abs(pitch) > np.pi*2:
                roll = np.deg2rad(roll)
                pitch = np.deg2rad(pitch)
        # 랩핑
        roll = (roll + np.pi) % (2*np.pi) - np.pi
        pitch = (pitch + np.pi) % (2*np.pi) - np.pi
        return roll, pitch

    def run(self, state, command):
        """한 틱마다: trot step -> IMU 보정/FF -> 발 위치 반환 (+ 안전모드)"""
        # 현재 틱
        cur_tick = int(getattr(state, "ticks", 0))

        vx, vy, wz = _get_velocity3(command)
        command.velocity = [np.clip(vx, -self.max_x_vel,  self.max_x_vel),
                            np.clip(vy, -self.max_y_vel,  self.max_y_vel),
                            np.clip(wz, -self.max_yaw_rate, self.max_yaw_rate)]
        command.yaw_rate = command.velocity[2]

        # 기본 게이트 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height
        new_foot_locations = state.foot_location.copy()

        # --- IMU 읽기 + 단위/랩핑 가드  # <<< safety
        roll_raw = float(getattr(state, "imu_roll", 0.0))
        pitch_raw = float(getattr(state, "imu_pitch", 0.0))
        inverted = bool(getattr(state, "is_inverted", False))
        roll, pitch = self._normalize_angles_and_units(roll_raw, pitch_raw)

        # --- 안전모드 판정  # <<< safety
        tip = (abs(roll) > np.deg2rad(self.tip_roll_deg)) or (abs(pitch) > np.deg2rad(self.tip_pitch_deg)) or inverted
        hard_tip = (abs(roll) > np.deg2rad(self.hard_tip_deg)) or (abs(pitch) > np.deg2rad(self.hard_tip_deg))
        if tip or hard_tip:
            hold = max(1, int(self.safemode_hold_sec / max(self.time_step, 1e-3)))
            self._safe_mode_until_tick = cur_tick + hold

        safe_mode = (cur_tick <= self._safe_mode_until_tick)

        # === (A) IMU 보정(완화/부드럽게) ===
        apply_imu = (self.use_imu and (not safe_mode))  # <<< safety: 안전모드에선 IMU 보정 끔
        if apply_imu:
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
                    is_stance = self._get_in_contact(state, leg_index, command.robot_height)

                    dz = 0.0
                    if is_stance:
                        new_z = command.robot_height * cp * cr
                        dz = -1.0 * (command.robot_height - new_z)

                    new_foot_locations[2, leg_index] += dz
                    

                    # swing에는 더 약하게
                    xy_gain = 1.0 if is_stance else 0.2
                    z_now_after = new_foot_locations[2, leg_index]
                    dx = (-1.0 * z_now_after) * tp * xy_gain
                    dy = ( 1.0 * z_now_after) * tr * xy_gain
                    new_foot_locations[0, leg_index] += dx
                    new_foot_locations[1, leg_index] += dy

        # === (B) rear 스윙 FF 피치/낙하: 안전모드면 미적용  # <<< safety
        skip_ff = safe_mode
        if not skip_ff:
            try:
                contact_modes = self.contacts(int(getattr(state, "ticks", 0)))
            except Exception:
                contact_modes = [1, 1, 1, 1]

            rr_swing = (contact_modes[2] == 0)
            rl_swing = (contact_modes[3] == 0)
            rear_swinging = rr_swing or rl_swing

            if rear_swinging:
                swing_prop = float(self.subphase_ticks(state.ticks)) / float(max(self.swing_ticks, 1))
                s0, s1 = self.ff_ramp_start, self.ff_ramp_end
                if swing_prop <= s0: r = 0.0
                elif swing_prop >= s1: r = 1.0
                else: r = (swing_prop - s0) / max((s1 - s0), 1e-6)

                p_ff = -np.deg2rad(self.ff_pitch_deg) * r
                t_pitch = np.tan(p_ff)
                dz_drop = self.ff_height_drop * r

                only_one_rear_swing = (rr_swing ^ rl_swing)
                if only_one_rear_swing:
                    p_ff   *= self.ff_pitch_boost_when_single_rear
                    t_pitch = np.tan(p_ff)
                    dz_drop *= self.ff_drop_boost_when_single_rear

                for i in range(4):
                    x = new_foot_locations[0, i]
                    new_foot_locations[2, i] += x * t_pitch

                new_foot_locations[2, :] -= dz_drop

                z_soft = command.robot_height - 200
                margin = 10
                z = new_foot_locations[2, :]
                mask = z < (z_soft + margin)
                if np.any(mask):
                    new_foot_locations[2, mask] = z_soft + margin - 0.5 * ((z_soft + margin) - z[mask])
                new_foot_locations[2, :] = np.maximum(new_foot_locations[2, :], z_soft + margin)

        # <<< safety: 안전모드 동작(정지/고도상향/XY감속)
        if safe_mode:
            # 속도 즉시 0
            command.velocity = [0.0, 0.0, 0.0]
            command.yaw_rate = 0.0
            # IMU 명령 잔류 제거
            self._lp_roll_cmd = 0.0
            self._lp_pitch_cmd = 0.0
            # 스윙 기본값 강화
            self.swingController._failsafe_min_swing = 120
            self.swingController._failsafe_extra_clearance = 0.06
            self.swingController._failsafe_xy_scale = 0.5
        else:
            self.swingController._failsafe_min_swing = None
            self.swingController._failsafe_extra_clearance = None
            self.swingController._failsafe_xy_scale = None

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
    front_lift_gain = 1.15
    rear_lift_gain  = 1.20
    extra_clearance = 30  

    # 새로 전달받는 보장값(stairs)
    min_step_length = 120  
    min_swing_lift  = 80  

    # === 뒷다리 전진 램프 back-off/forward profile parameters ===
    rear_backoff_dx    = -20
    rear_backoff_lift  = 20
    rear_backoff_start = 0.00
    rear_backoff_peak  = 0.30
    rear_backoff_end   = 0.65

    # >>> 앞다리도 코 간섭 방지 back-off/forward
    front_backoff_dx    = -20
    front_backoff_lift  = 8
    front_backoff_start = 5
    front_backoff_peak  = 0.25
    front_backoff_end   = 0.55

    # >>> 앞다리 late-XY 램프/브레이크
    front_xy_ramp_start = 0.40
    front_brake_start   = 0.85
    front_brake_ratio   = 0.25

    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)

        # 두 번째 뒷발 부스트 상태
        self._prev_tick = -1
        self._prev_contact = np.array([1, 1, 1, 1], dtype=int)
        self._second_rear_boost_leg = None
        self._second_rear_boost_expire = -1
        self.second_rear_boost_gain = 1.50
        self.second_rear_boost_window = int(0.35 * self.swing_ticks)

        # 적응형 z(앞/뒤)
        self._rear_extra_z = np.zeros(4, dtype=float)
        self._rear_contact_stuck = np.zeros(4, dtype=int)
        self.rear_extra_z_gain = 10
        self.rear_extra_z_max  = 100
        self.rear_extra_z_decay = 0.92
        self.rear_contact_stuck_thresh = 1

        self._front_extra_z = np.zeros(4, dtype=float)
        self._front_contact_stuck = np.zeros(4, dtype=int)
        self.front_extra_z_gain = 25
        self.front_extra_z_max  = 45
        self.front_extra_z_decay = 0.85
        self.front_contact_stuck_thresh = 2

        # 앞발 z-정점에서 x 전진 부스트
        self.front_apex_x_boost_gain  = 0.08
        self.front_apex_x_boost_sigma = 0.12

        # (상위에서 내려오는 휴리스틱 파라미터)
        self.use_contact_heuristic = getattr(self, "use_contact_heuristic", True)
        self.heuristic_z_soft = getattr(self, "heuristic_z_soft", 200)
        self.heuristic_z_margin = getattr(self, "heuristic_z_margin", 20)

        # <<< safety: 상위 failsafe 값 전달용(없으면 None)
        self._failsafe_min_swing = None
        self._failsafe_extra_clearance = None
        self._failsafe_xy_scale = None

    def _get_in_contact(self, state, leg_index, robot_height):
        if hasattr(state, "contact"):
            try:
                v = int(state.contact[leg_index])
                if v in (0, 1):
                    return bool(v)
            except Exception:
                pass
        if getattr(self, "use_contact_heuristic", False):
            try:
                z_soft = robot_height - float(getattr(self, "heuristic_z_soft", 200))
                margin = float(getattr(self, "heuristic_z_margin", 10))
                foot_z = float(state.foot_location[2, leg_index])
                return foot_z < (z_soft + margin)
            except Exception:
                return False
        return False

    rear_lift_delay = 0.40
    rear_lift_full  = 0.80

    def _clamp01(self, x):
        return max(0.0, min(1.0, x))

    def _ramp(self, s0, s1, s):
        if s <= s0: return 0.0
        if s >= s1: return 1.0
        return (s - s0) / max(s1 - s0, 1e-6)

    def swing_height(self, swing_phase, leg_index=None, in_contact=False):
        # 기본 삼각 프로파일
        if swing_phase < 0.5:
            swing_h = (swing_phase / 0.5) * self.z_leg_lift
        else:
            swing_h = self.z_leg_lift * (1.0 - (swing_phase - 0.5) / 0.5)

        # 앞/뒤 다리 별 게인
        if leg_index is not None:
            if leg_index in self.front_leg_indices:
                swing_h *= self.front_lift_gain
                s = swing_phase
                up   = self._ramp(self.front_backoff_start, self.front_backoff_peak, s)
                down = 1.0 - self._ramp(self.front_backoff_peak, self.front_backoff_end, s)
                bump = min(up, down)
                swing_h += self.front_backoff_lift * bump
            elif leg_index in self.rear_leg_indices:
                if swing_phase <= self.rear_lift_delay:
                    gain = 1.0
                elif swing_phase >= self.rear_lift_full:
                    gain = self.rear_lift_gain
                else:
                    t = ((swing_phase - self.rear_lift_delay) /
                         max(self.rear_lift_full - self.rear_lift_delay, 1e-6))
                    gain = 1.0 + t * (self.rear_lift_gain - 1.0)
                swing_h *= gain

        # 스윙 중 접촉이면 여유고도 추가
        if in_contact:
            swing_h += self.extra_clearance

        # rear z-plateau
        if leg_index in self.rear_leg_indices:
            s = swing_phase
            if 0.45 < s < 0.60:
                w = 1.0 - abs((s - 0.525) / 0.075)
                swing_h += 10 * w

        # 적응형 z
        if leg_index in self.rear_leg_indices:
            swing_h += self._rear_extra_z[leg_index]
        if leg_index in self.front_leg_indices:
            swing_h += self._front_extra_z[leg_index]

        # --- 최소 스윙 높이 강제 ---
        min_z = self.min_swing_lift
        if self._failsafe_min_swing is not None:          # <<< safety
            min_z = max(min_z, self._failsafe_min_swing)
        swing_h = max(swing_h, min_z)
        return swing_h

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        vx, vy, wz = _get_velocity3(command)

        if swing_phase < 0.5:
            return self.default_stance[:, leg_index]
        else:
            base_delta_2d = 2.6 * np.array([vx, vy]) * self.phase_length * self.time_step

            lam_start = 0.3
            lam = (swing_phase - lam_start) / (1.0 - lam_start)
            lam = max(0.0, min(1.0, lam))

            step_x_min = 20
            sign = np.sign(vx) if abs(vx) > 0.03 else 0.0

            step_x_target = (1.0 - lam) * base_delta_2d[0] + lam * (sign * step_x_min)
            delta_pos = np.array([step_x_target, base_delta_2d[1], 0.0])

            theta = self.stance_ticks * self.time_step * wz * 2.0
            rotation = rotz(theta)
            return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def _update_second_rear_boost_flag(self, state):
        try:
            cur_tick = int(state.ticks)
            if cur_tick == self._prev_tick:
                return
        except Exception:
            return

        current_contact = np.array([
            self._get_in_contact(state, i, state.robot_height) for i in range(4)
        ], dtype=int)

        rear_indices = self.rear_leg_indices
        for leg_idx in rear_indices:
            if self._prev_contact[leg_idx] == 0 and current_contact[leg_idx] == 1:
                other_rear_idx = rear_indices[0] if leg_idx == rear_indices[1] else rear_indices[1]
                self._second_rear_boost_leg = other_rear_idx
                self._second_rear_boost_expire = cur_tick + self.second_rear_boost_window

        self._prev_contact = current_contact
        self._prev_tick = cur_tick

    def next_foot_location(self, swing_prop, leg_index, state, command):
        foot_matrix = np.asarray(state.foot_location, dtype=float)
        foot_location = foot_matrix[:, leg_index]

        swing_prop += (1.0 / self.swing_ticks)

        # 최근 뒷발 착지 이벤트 갱신
        self._update_second_rear_boost_flag(state)

        in_contact = self._get_in_contact(state, leg_index, command.robot_height)

        swing_h = self.swing_height(swing_prop, leg_index=leg_index, in_contact=in_contact)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, swing_prop)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        velocity_xy = (touchdown_location - foot_location) / float(max(time_left, 1e-6)) * np.array([1.0, 1.0, 0.0])
        delta_xy = velocity_xy * self.time_step

        # rear back-off 미분 효과
        if leg_index in self.rear_leg_indices:
            up   = self._ramp(self.rear_backoff_start, self.rear_backoff_peak, swing_prop)
            down = 1.0 - self._ramp(self.rear_backoff_peak, self.rear_backoff_end, swing_prop)
            bump = min(up, down)

            backoff_offset_x = self.rear_backoff_dx * bump
            next_s = min(1.0, swing_prop + (1.0 / self.swing_ticks))
            up_n   = self._ramp(self.rear_backoff_start, self.rear_backoff_peak, next_s)
            down_n = 1.0 - self._ramp(self.rear_backoff_peak, self.rear_backoff_end, next_s)
            bump_n = min(up_n, down_n)
            backoff_offset_x_next = self.rear_backoff_dx * bump_n
            backoff_dx_step = (backoff_offset_x_next - backoff_offset_x)
            delta_xy += np.array([backoff_dx_step, 0.0, 0.0])

            lam = self._clamp01((swing_prop - 0.5) / 0.5)
            delta_xy *= (0.6 + 0.4 * lam)
            mid = np.exp(-((swing_prop - 0.55)**2) / (2 * 0.10**2))
            delta_xy[0] *= (1.0 + self.second_rear_boost_gain * mid)

            brake = self._clamp01((swing_prop - 0.70) / 0.25)
            delta_xy *= (1.0 - 0.4 * brake)

            other_rear = 3 if leg_index == 2 else 2
            other_in_contact = self._get_in_contact(state, other_rear, command.robot_height)
            if not other_in_contact:
                delta_xy *= 0.7
                swing_h += 15

            # 스윙 중 접촉 기반 적응형 z
            if (0.25 < swing_prop < 0.85):
                if in_contact:
                    self._rear_contact_stuck[leg_index] += 1
                    if self._rear_contact_stuck[leg_index] >= self.rear_contact_stuck_thresh:
                        self._rear_extra_z[leg_index] = min(
                            self.rear_extra_z_max,
                            self._rear_extra_z[leg_index] + self.rear_extra_z_gain
                        )
                        self._rear_contact_stuck[leg_index] = 0
                else:
                    if self._rear_contact_stuck[leg_index] > 0:
                        self._rear_contact_stuck[leg_index] -= 1

            self._rear_extra_z[leg_index] *= self.rear_extra_z_decay

        # 앞다리 back-off + late 램프/브레이크
        if leg_index in self.front_leg_indices:
            up_f   = self._ramp(self.front_backoff_start, self.front_backoff_peak, swing_prop)
            down_f = 1.0 - self._ramp(self.front_backoff_peak, self.front_backoff_end, swing_prop)
            bump_f = min(up_f, down_f)

            front_backoff_offset_x = self.front_backoff_dx * bump_f
            next_s = min(1.0, swing_prop + (1.0 / self.swing_ticks))
            up_fn   = self._ramp(self.front_backoff_start, self.front_backoff_peak, next_s)
            down_fn = 1.0 - self._ramp(self.front_backoff_peak, self.front_backoff_end, next_s)
            bump_fn = min(up_fn, down_fn)
            front_backoff_offset_x_next = self.front_backoff_dx * bump_fn
            front_backoff_dx_step = (front_backoff_offset_x_next - front_backoff_offset_x)
            delta_xy += np.array([front_backoff_dx_step, 0.0, 0.0])

            lam_f = self._clamp01((swing_prop - self.front_xy_ramp_start) / (1.0 - self.front_xy_ramp_start))
            delta_xy *= (0.3 + 0.7 * lam_f)

            brake_f = self._clamp01((swing_prop - self.front_brake_start) / (1.0 - self.front_brake_start))
            delta_xy *= (1.0 - self.front_brake_ratio * brake_f)

            mid_front = np.exp(-((swing_prop - 0.5)**2) / (2 * 0.10**2))
            delta_xy[0] *= (1.0 + 0.25 * mid_front)

            if (0.25 < swing_prop < 0.85):
                if in_contact:
                    self._front_contact_stuck[leg_index] += 1
                    if self._front_contact_stuck[leg_index] >= self.front_contact_stuck_thresh:
                        self._front_extra_z[leg_index] = min(
                            self.front_extra_z_max,
                            self._front_extra_z[leg_index] + self.front_extra_z_gain
                        )
                        self._front_contact_stuck[leg_index] = 0
                else:
                    if self._front_contact_stuck[leg_index] > 0:
                        self._front_contact_stuck[leg_index] -= 1
            self._front_extra_z[leg_index] *= self.front_extra_z_decay

        # 두 번째 뒷발 X 델타 부스트
        if leg_index in self.rear_leg_indices:
            try:
                cur_tick = int(getattr(state, "ticks"))
            except Exception:
                cur_tick = -1
            if (self._second_rear_boost_leg == leg_index) and (cur_tick <= self._second_rear_boost_expire):
                mid = np.exp(-((swing_prop - 0.5)**2) / (2 * 0.12**2))
                delta_xy[0] *= (self.second_rear_boost_gain * (1.0 + 0.15 * mid))

        # <<< safety: 안전모드면 XY 감속
        if self._failsafe_xy_scale is not None:
            delta_xy *= float(self._failsafe_xy_scale)

        # 마지막 틱: 정확히 터치다운 z
        if swing_prop >= 1.0:
            new_position = touchdown_location * np.array([1.0, 1.0, 0.0]) + np.array([0.0, 0.0, command.robot_height])
            z_soft = command.robot_height - 200
            margin = 10
            new_position[2] = max(new_position[2], z_soft + margin)
            return new_position

        z_vec = np.array([0.0, 0.0, swing_h + command.robot_height])

        # LOG: z-soft-limit
        try:
            z_min_soft = command.robot_height - 200
            z_leg = foot_location[2] + (swing_h)
            if z_leg < z_min_soft + 10:
                print(f"[Tick {int(getattr(state,'ticks',-1))}] z-soft-limit near: leg={leg_index}, z≈{z_leg:.3f}")
        except Exception:
            pass

        out = foot_location * np.array([1.0, 1.0, 0.0]) + z_vec + delta_xy
        z_soft = command.robot_height - 200
        out[2] = max(out[2], z_soft + 10)

        # <<< safety: 안전모드면 여유고도 더 추가
        if self._failsafe_extra_clearance is not None:
            out[2] += float(self._failsafe_extra_clearance)

        return out
