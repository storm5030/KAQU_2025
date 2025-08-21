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
        # self.min_swing_lift  = self.stair_rise  + self.lift_margin   # 0.08 m (원본)
        self.min_swing_lift  = 0.10   # <<< changed: 0.08 → 0.10 (계단에서 기본 스윙 높이 여유)

        # z 에러 게인 (기존 설계 유지)
        z_error_constant = 0.5 * 4

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
        # self.imu_follow_gain = 0.12
        # self.imu_max_corr = 0.15
        # self.imu_lpf_alpha = 0.2
        self.imu_follow_gain = 0.08   # <<< changed
        self.imu_max_corr    = 0.10   # <<< changed
        self.imu_lpf_alpha   = 0.3    # <<< changed
        self._lp_roll_cmd = 0.0
        self._lp_pitch_cmd = 0.0

        # === 다리 인덱스 순서: FR, FL, RR, RL = [0, 1, 2, 3] ===
        # 앞다리(Front): FR, FL -> [0, 1]
        # 뒷다리(Rear):  RR, RL -> [2, 3]
        self.front_leg_indices = getattr(leg.stair, "front_leg_indices", [0, 1])
        self.rear_leg_indices  = getattr(leg.stair, "rear_leg_indices",  [2, 3])

        # 앞다리 스윙 높이 게인(요청사항: 앞다리 gain으로 조정)
        # self.front_lift_gain = getattr(leg.stair, "front_lift_gain", 1.40)
        # self.rear_lift_gain  = getattr(leg.stair, "rear_lift_gain", 1.70)
        self.front_lift_gain = getattr(leg.stair, "front_lift_gain", 1.55)  # <<< changed
        self.rear_lift_gain  = getattr(leg.stair, "rear_lift_gain", 1.85)   # <<< changed

        # 스윙 중인데 접촉이 남아있으면 여유고도(미터)
        # self.extra_clearance = getattr(leg.stair, "extra_clearance", 0.03)
        self.extra_clearance = getattr(leg.stair, "extra_clearance", 0.05)   # <<< changed

        # SwingController에 전달 (최소 보장값 포함!)
        self.swingController.front_leg_indices = self.front_leg_indices
        self.swingController.rear_leg_indices  = self.rear_leg_indices
        self.swingController.front_lift_gain   = self.front_lift_gain
        self.swingController.rear_lift_gain    = self.rear_lift_gain
        self.swingController.extra_clearance   = self.extra_clearance
        self.swingController.min_step_length   = self.min_step_length
        self.swingController.min_swing_lift    = self.min_swing_lift

        # --- FF(선행) 피치 보정 파라미터 ---
        # self.ff_pitch_deg = 11.0
        # self.ff_ramp_start = 0.15
        # self.ff_ramp_end   = 0.95
        # self.ff_height_drop = 0.010
        self.ff_pitch_deg = 7.0       # <<< changed (과도 숙임 방지)
        self.ff_ramp_start = 0.20     # <<< changed
        self.ff_ramp_end   = 0.90     # <<< changed
        self.ff_height_drop = 0.004   # <<< changed

        # "두 번째 뒷발" 부스트 완화
        self.ff_pitch_boost_when_single_rear = 1.00  # <<< changed (부스트 해제)
        self.ff_drop_boost_when_single_rear  = 1.00  # <<< changed

        # === (추가) 접촉 휴리스틱 파라미터 ===
        self.use_contact_heuristic = True
        self.heuristic_z_soft = 0.20   # 바디 기준 하한(예: -20cm)
        self.heuristic_z_margin = 0.01 # 여유 1cm

        # SwingController에도 공유(동일 값 사용)
        self.swingController.use_contact_heuristic = self.use_contact_heuristic
        self.swingController.heuristic_z_soft = self.heuristic_z_soft
        self.swingController.heuristic_z_margin = self.heuristic_z_margin

    # --- (추가) contact 추정 유틸: state.contact가 없거나 값이 이상하면 z-기반 휴리스틱 ---
    def _get_in_contact(self, state, leg_index, robot_height):
        # 1) 실측 contact가 있으면 그대로 사용(0/1만 유효)
        if hasattr(state, "contact"):
            try:
                v = int(state.contact[leg_index])
                if v in (0, 1):
                    return bool(v)
            except Exception:
                pass
        # 2) 휴리스틱: 발 z가 바디 기준 하한 근처면 접촉으로 간주
        if getattr(self, "use_contact_heuristic", False):
            try:
                z_soft = robot_height - float(getattr(self, "heuristic_z_soft", 0.20))
                margin = float(getattr(self, "heuristic_z_margin", 0.01))
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
                    # (변경) is_stance 판단: contact 없으면 z-휴리스틱
                    is_stance = self._get_in_contact(state, leg_index, command.robot_height)

                    # z 보정: stance 위주
                    dz = 0.0
                    if is_stance:
                        new_z = command.robot_height * cp * cr
                        dz = -1.0 * (command.robot_height - new_z)

                    new_foot_locations[2, leg_index] += dz

                    # x,y 보정: swing에는 약하게
                    # xy_gain = 1.0 if is_stance else 0.3
                    xy_gain = 1.0 if is_stance else 0.2   # <<< changed (스윙 중 XY 끌림 더 약화)
                    z_now_after = new_foot_locations[2, leg_index]
                    dx = (-1.0 * z_now_after) * tp * xy_gain
                    dy = ( 1.0 * z_now_after) * tr * xy_gain

                    new_foot_locations[0, leg_index] += dx
                    new_foot_locations[1, leg_index] += dy

        # === (B) rear 스윙시에만 FF(선행) 피치로 전방 숙이기 + COM 살짝 낮추기 ===
        # 현재 phase 접지 패턴: [FR, FL, RR, RL] = [0,1,2,3]
        try:
            contact_modes = self.contacts(int(getattr(state, "ticks", 0)))
        except Exception:
            contact_modes = [1, 1, 1, 1]  # 임시 폴백

        rr_swing = (contact_modes[2] == 0)   # 어떤 발이 스윙중인지 확인
        rl_swing = (contact_modes[3] == 0)
        rear_swinging = rr_swing or rl_swing

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

            # 목표 피치(라디안): "앞으로 숙이기"
            p_ff = -np.deg2rad(self.ff_pitch_deg) * r
            t_pitch = np.tan(p_ff)
            # COM 살짝 낮추기(과하면 충돌/미끄럼 가능, 0~1cm 권장)
            dz_drop = self.ff_height_drop * r

            # 이번 틱에 "한쪽 뒷발만" 스윙이면 (완화: 부스트 해제)
            only_one_rear_swing = (rr_swing ^ rl_swing)
            if only_one_rear_swing:
                p_ff   *= self.ff_pitch_boost_when_single_rear
                t_pitch = np.tan(p_ff)  # 재계산
                dz_drop *= self.ff_drop_boost_when_single_rear

            # z += x * tan(pitch)
            for i in range(4):
                x = new_foot_locations[0, i]
                new_foot_locations[2, i] += x * t_pitch

            # COM 살짝 낮추기
            new_foot_locations[2, :] -= dz_drop
            # ---z 하한 소프트 클립으로 포화 패턴 완화 ---
            z_soft = command.robot_height - 0.20
            margin = 0.010                        # <<< changed: 5mm → 10mm
            z = new_foot_locations[2, :]
            mask = z < (z_soft + margin)
            if np.any(mask):
                new_foot_locations[2, mask] = z_soft + margin - 0.5 * ((z_soft + margin) - z[mask])

            # 추가: 전체에 대해 소프트 하한 가드 한 번 더 적용  # <<< changed
            new_foot_locations[2, :] = np.maximum(new_foot_locations[2, :], z_soft + margin)

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
    extra_clearance = 0.03  # m

    # 새로 전달받는 보장값(stairs)
    min_step_length = 0.12  # m
    min_swing_lift  = 0.08  # m

    # === 뒷다리 전진 램프 back-off/forward profile parameters ===
    rear_backoff_dx    = -0.07  # m (초반에 뒤로 당길 최대치; 음수=뒤쪽)
    rear_backoff_lift  = 0.020  # m (back-off 동안 추가 z)
    rear_backoff_start = 0.00   # 스윙 진행률 시작
    rear_backoff_peak  = 0.35   # 여기까지 back-off ↑
    rear_backoff_end   = 0.70   # 여기까지 back-off ↓(이후 0)

    # >>> 앞다리도 코 간섭 방지를 위한 back-off/forward 프로파일
    front_backoff_dx    = -0.06  # <<< changed: -0.04 → -0.06 (살짝 더 뒤로)
    front_backoff_lift  = 0.010
    front_backoff_start = 0.05
    front_backoff_peak  = 0.30
    front_backoff_end   = 0.60

    # >>> 앞다리 late-XY 램프/브레이크
    front_xy_ramp_start = 0.55   # <<< changed: 0.45 → 0.55 (가속을 더 늦게)
    front_brake_start   = 0.80   # <<< changed: 0.85 → 0.80 (브레이크 조금 일찍)
    front_brake_ratio   = 0.6    # <<< changed: 0.5 → 0.6 (감속 더 강하게)

    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)

        # >>>두 번째 뒷발 부스트를 위한 상태 변수
        self._prev_tick = -1
        self._prev_contact = np.array([1, 1, 1, 1], dtype=int)
        self._second_rear_boost_leg = None        # 2(RR) 또는 3(RL)
        self._second_rear_boost_expire = -1       # 부스트 만료 tick
        self.second_rear_boost_gain = 1.50        # X 델타 배수 (원본보다 소폭 강화되어 있었음)
        self.second_rear_boost_window = int(0.35 * self.swing_ticks)  # 착지 후 ~35% 스윙창

        # >>> 못 올라오는 '특정 뒷발'만 z를 더 주는 적응형 부스트 상태/파라미터
        self._rear_extra_z = np.zeros(4, dtype=float)   # 다리별 추가 z (m)
        self._rear_contact_stuck = np.zeros(4, dtype=int)  # 스윙 중 접촉 카운트
        self.rear_extra_z_gain = 0.025
        self.rear_extra_z_max  = 0.040
        self.rear_extra_z_decay = 0.85
        self.rear_contact_stuck_thresh = 2

        # >>> (front adaptive z): 못 올라오는 '특정 앞발'만 z를 더 주는 상태/파라미터
        self._front_extra_z = np.zeros(4, dtype=float)       # 다리별 추가 z (m) - 앞다리 인덱스만 사용
        self._front_contact_stuck = np.zeros(4, dtype=int)   # 스윙 중 접촉 카운트
        self.front_extra_z_gain = 0.020
        self.front_extra_z_max  = 0.035
        self.front_extra_z_decay = 0.85
        self.front_contact_stuck_thresh = 2

        # >>>  앞발 z-정점에서 x 전진 부스트 파라미터
        self.front_apex_x_boost_gain  = 0.05
        self.front_apex_x_boost_sigma = 0.15

        # === (추가) 접촉 휴리스틱 파라미터(상위에서 전달됨) ===
        self.use_contact_heuristic = getattr(self, "use_contact_heuristic", True)
        self.heuristic_z_soft = getattr(self, "heuristic_z_soft", 0.20)
        self.heuristic_z_margin = getattr(self, "heuristic_z_margin", 0.01)

    # --- (추가) contact 추정 유틸: state.contact가 없거나 값이 이상하면 z-기반 휴리스틱 ---
    def _get_in_contact(self, state, leg_index, robot_height):
        # 1) 실측 contact가 있으면 그대로 사용(0/1만 유효)
        if hasattr(state, "contact"):
            try:
                v = int(state.contact[leg_index])
                if v in (0, 1):
                    return bool(v)
            except Exception:
                pass
        # 2) 휴리스틱: 발 z가 바디 기준 하한 근처면 접촉으로 간주
        if getattr(self, "use_contact_heuristic", False):
            try:
                z_soft = robot_height - float(getattr(self, "heuristic_z_soft", 0.20))
                margin = float(getattr(self, "heuristic_z_margin", 0.01))
                foot_z = float(state.foot_location[2, leg_index])
                return foot_z < (z_soft + margin)
            except Exception:
                return False
        return False

    rear_lift_delay = 0.35  # 이 구간 전에는 증폭 X
    rear_lift_full  = 0.70  # 이 이후부터 full rear_lift_gain

    def _clamp01(self, x):
        return max(0.0, min(1.0, x))

    def _ramp(self, s0, s1, s):
        """s0~s1에서 0→1 선형 램프"""
        if s <= s0: return 0.0
        if s >= s1: return 1.0
        return (s - s0) / max(s1 - s0, 1e-6)

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
                # 앞다리 back-off 구간엔 z 여유 조금 추가
                s = swing_phase
                up   = self._ramp(self.front_backoff_start, self.front_backoff_peak, s)
                down = 1.0 - self._ramp(self.front_backoff_peak, self.front_backoff_end, s)
                bump = min(up, down)
                swing_h += self.front_backoff_lift * bump
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

        # rear z-plateau (코 넘김 여유)
        if leg_index in self.rear_leg_indices:
            s = swing_phase
            if 0.45 < s < 0.60:
                w = 1.0 - abs((s - 0.525) / 0.075)  # 0→1→0
                swing_h += 0.005 * w

        # 못 올라오는 '특정 뒷발' 적응형 z
        if leg_index in self.rear_leg_indices:
            swing_h += self._rear_extra_z[leg_index]

        # (front adaptive z)
        if leg_index in self.front_leg_indices:
            swing_h += self._front_extra_z[leg_index]

        # --- 최소 스윙 높이 강제 (계단 높이+여유) ---
        swing_h = max(swing_h, self.min_swing_lift)
        return swing_h

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        vx, vy, wz = _get_velocity3(command)

        if swing_phase < 0.5:
            # 초반은 제자리 근처에서 들어올리기
            return self.default_stance[:, leg_index]
        else:
            # 기본: 2 * v * T_half
            base_delta_2d = 2 * np.array([vx, vy]) * self.phase_length * self.time_step

            # 최소 보폭(계단용): 후반으로 갈수록 강하게 적용
            lam_start = 0.35
            lam = (swing_phase - lam_start) / (1.0 - lam_start)
            lam = max(0.0, min(1.0, lam))

            # 목표 step_x: base와 최소보폭 사이 보간
            # step_x_min = self.min_step_length  # 원본 0.12
            step_x_min = 0.09  # <<< changed: 0.12 → 0.09 (넘겨찍기 방지)

            # vx가 거의 0이면 +방향 고정 대신 0.0으로
            sign = np.sign(vx) if abs(vx) > 0.03 else 0.0  # <<< changed

            step_x_target = (1.0 - lam) * base_delta_2d[0] + lam * (sign * step_x_min)
            delta_pos = np.array([step_x_target, base_delta_2d[1], 0.0])

            theta = self.stance_ticks * self.time_step * wz * 2.0
            rotation = rotz(theta)
            return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos


    def _update_second_rear_boost_flag(self, state):
        """
        매 틱마다 이전 contact 상태와 현재를 비교하여,
        한쪽 뒷발이 막 착지했을 때 '다음' 뒷발에 부스트 플래그를 설정한다.
        """
        try:
            cur_tick = int(state.ticks)
            # 이전 틱과 같으면 중복 실행 방지
            if cur_tick == self._prev_tick:
                return
        except Exception:
            # state.ticks가 없으면 실행 불가
            return

        # 현재 4개 다리의 접촉 상태를 가져옴
        current_contact = np.array([
            self._get_in_contact(state, i, state.robot_height) for i in range(4)
        ], dtype=int)

        # 뒷다리 인덱스: 2(RR), 3(RL)
        rear_indices = self.rear_leg_indices

        # 뒷다리가 (스윙 -> 접지)로 바뀐 순간을 감지
        for leg_idx in rear_indices:
            # 이전에는 스윙(0)이었고, 지금은 접지(1) 상태인가?
            if self._prev_contact[leg_idx] == 0 and current_contact[leg_idx] == 1:
                # 그렇다면, 다른 쪽 뒷발이 바로 '두 번째 뒷발'이 된다.
                other_rear_idx = rear_indices[0] if leg_idx == rear_indices[1] else rear_indices[1]

                # 부스트 대상 다리와 부스트가 유효한 시간(tick)을 설정
                self._second_rear_boost_leg = other_rear_idx
                self._second_rear_boost_expire = cur_tick + self.second_rear_boost_window

        # 현재 상태를 다음 틱을 위해 저장
        self._prev_contact = current_contact
        self._prev_tick = cur_tick

    def next_foot_location(self, swing_prop, leg_index, state, command):
        foot_matrix = np.asarray(state.foot_location, dtype=float)
        foot_location = foot_matrix[:, leg_index]

        swing_prop += (1.0 / self.swing_ticks)

        # 매 틱 최초 호출에서 최근 뒷발 착지 이벤트 갱신
        self._update_second_rear_boost_flag(state)

        # (변경) in_contact 판단: contact 없으면 z-휴리스틱
        in_contact = self._get_in_contact(state, leg_index, command.robot_height)

        swing_h = self.swing_height(swing_prop, leg_index=leg_index, in_contact=in_contact)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, swing_prop)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)

        velocity_xy = (touchdown_location - foot_location) / float(max(time_left, 1e-6)) * np.array([1.0, 1.0, 0.0])
        delta_xy = velocity_xy * self.time_step

        # === rear 전용 back-off(초반 뒤로 → 중반 복귀)의 미분 효과를 속도로 반영 ===
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

            # rear late-XY 램프: 0.5~1.0에서 0.3배→1.0배
            lam = self._clamp01((swing_prop - 0.5) / 0.5)
            delta_xy *= (0.4 + 0.6 * lam)
            # z 최대 부근(≈0.5)에서 x를 한 번 더 밀어주는 가우시안 부스트
            mid = np.exp(-((swing_prop - 0.5)**2) / (2 * 0.12**2))
            delta_xy[0] *= (1.0 + 0.45 * mid)

            # 터치다운 직전 브레이크, 속도감소 (완화 강화)
            # brake = self._clamp01((swing_prop - 0.85) / 0.15)
            # delta_xy *= (1.0 - 0.5 * brake)
            brake = self._clamp01((swing_prop - 0.80) / 0.20)  # <<< changed
            delta_xy *= (1.0 - 0.6 * brake)                    # <<< changed

            other_rear = 3 if leg_index == 2 else 2
            other_in_contact = self._get_in_contact(state, other_rear, command.robot_height)

            # 반대쪽 뒷발이 아직 스윙(=지지 아님)인 동안은 더 보수적으로:
            if not other_in_contact:
                delta_xy *= 0.7
                swing_h += 0.008  # 6~12 mm

            # >>> 스윙 중 접촉 기반의 '특정 뒷발' z 부스트 감지/갱신
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

            # 매 틱 감쇠(과도한 상승 방지)
            self._rear_extra_z[leg_index] *= self.rear_extra_z_decay

        # >>> 앞다리 전용 back-off(초반 살짝 뒤로 → 중반 복귀) + late 램프/브레이크
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

            # 앞다리도 후반에 XY 램프 인가(앞계단으로 넘어갈 때 가속)
            lam_f = self._clamp01((swing_prop - self.front_xy_ramp_start) / (1.0 - self.front_xy_ramp_start))
            delta_xy *= (0.3 + 0.7 * lam_f)

            # 터치다운 직전엔 감속 (강화)
            brake_f = self._clamp01((swing_prop - self.front_brake_start) / (1.0 - self.front_brake_start))
            delta_xy *= (1.0 - self.front_brake_ratio * brake_f)

            # z-정점(≈0.5)에서만 X 전진 푸시 (약간 약화)
            mid_front = np.exp(-((swing_prop - 0.5)**2) / (2 * 0.10**2))
            delta_xy[0] *= (1.0 + 0.25 * mid_front)  # <<< changed: 0.35 → 0.25

            # 디버그: z-정점 근처에서 실제 푸시가 들어가는지 로그
            if 0.45 < swing_prop < 0.55:
                print(f"[Tick {int(getattr(state,'ticks',-1))}] front z-peak push: "
                      f"leg={leg_index}, mid≈{mid_front:.2f}, dX≈{float(delta_xy[0]):.3f}, "
                      f"extraZ={float(self._front_extra_z[leg_index]):.3f}")

            # >>> (front adaptive z): 스윙 중 접촉 기반의 '특정 앞발' z 부스트 갱신
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

            # 매 틱 감쇠(과도한 상승 방지)
            self._front_extra_z[leg_index] *= self.front_extra_z_decay

            # 디버그: 앞발 스윙 중 접촉 누적 상태 확인
            if (0.25 < swing_prop < 0.85) and in_contact:
                print(f"[Tick {int(getattr(state,'ticks',-1))}] front stuck contact: "
                      f"leg={leg_index}, cnt={int(self._front_contact_stuck[leg_index])}, "
                      f"extraZ={float(self._front_extra_z[leg_index]):.3f}")

            # 못 뜬 '특정 앞발'이면 z-정점(≈0.5) 부근에서 x를 더 밀어주기
            apex_w = np.exp(-((swing_prop - 0.5)**2) / (2 * (self.front_apex_x_boost_sigma ** 2)))
            if (self._front_extra_z[leg_index] > 0.0) or (self._front_contact_stuck[leg_index] > 0):
                delta_xy[0] += self.front_apex_x_boost_gain * apex_w

        # >>> '두 번째 뒷발'일 때 X 델타 부스트
        if leg_index in self.rear_leg_indices:
            try:
                cur_tick = int(getattr(state, "ticks"))
            except Exception:
                cur_tick = -1

            if (self._second_rear_boost_leg == leg_index) and (cur_tick <= self._second_rear_boost_expire):
                mid = np.exp(-((swing_prop - 0.5)**2) / (2 * 0.12**2))
                delta_xy[0] *= (self.second_rear_boost_gain * (1.0 + 0.15 * mid))

        # 마지막 틱: 정확히 터치다운 z로 세팅
        if swing_prop >= 1.0:
            new_position = touchdown_location * np.array([1.0, 1.0, 0.0]) + np.array([0.0, 0.0, command.robot_height])
            # 반환 전 z 소프트 하한 가드  # <<< changed
            z_soft = command.robot_height - 0.20
            margin = 0.010
            new_position[2] = max(new_position[2], z_soft + margin)
            return new_position

        z_vec = np.array([0.0, 0.0, swing_h + command.robot_height])

        # >>> LOG: limit margin
        try:
            z_min_soft = command.robot_height - 0.20
            z_leg = foot_location[2] + (swing_h)
            if z_leg < z_min_soft + 0.01:
                print(f"[Tick {int(getattr(state,'ticks',-1))}] z-soft-limit near: leg={leg_index}, z≈{z_leg:.3f}")
        except Exception:
            pass

        out = foot_location * np.array([1.0, 1.0, 0.0]) + z_vec + delta_xy
        # 반환 전 z 소프트 하한 가드  # <<< changed
        z_soft = command.robot_height - 0.20
        out[2] = max(out[2], z_soft + 0.010)
        return out
