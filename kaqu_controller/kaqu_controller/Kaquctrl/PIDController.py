#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType

class PID_controller(object):
    """
    Roll/Pitch 안정화용 PID
    - 입력: 라디안 (roll, pitch)
    - 출력: 라디안 보정각 [roll_corr, pitch_corr]
    - 포함: 각도 언랩, 측정값/미분 LPF, 데드밴드, 출력 제한, 앤티윈드업
    """
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        meas_tau: float = 0.15,   # 측정값 LPF 시정수 [s]
        d_tau: float = 0.05,      # 미분항 LPF 시정수 [s]
        out_limit: float = 0.20,  # 출력 제한 [rad]
        i_limit: float = 0.10,    # 적분항 제한 [rad]
        deadband_deg: float = 0.5 # 데드밴드 [deg]
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.meas_tau = float(max(meas_tau, 1e-6))
        self.d_tau = float(max(d_tau, 1e-6))
        self.out_limit = abs(out_limit)
        self.i_limit = abs(i_limit)
        self.deadband = np.deg2rad(abs(deadband_deg)) #라디안으로 저장

        self.desired_roll_pitch = np.array([0.0, 0.0]) #목표값
        self.clock = Clock(clock_type=ClockType.ROS_TIME)

        self.initialized = False
        self.reset()

    def reset(self):
        self.last_time = self.clock.now()
        self.last_error = np.array([0.0, 0.0])


        # LPF 상태
        self.meas_filt = np.array([0.0, 0.0]) #roll, pitch 필터한 값
        self.d_filt = np.array([0.0, 0.0]) #미분항 필터한 값


        #적분항 
        self.I_term = np.array([0.0, 0.0])
        self.initialized = False  # 첫 run에서 초기화

    def desired_RP_angles(self, des_roll: float, des_pitch: float):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])

    # [-pi, pi)로 래핑
    def _wrap_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi

    # a를 ref와 가장 가까운 동치각으로 이동
    def _nearest_angle(self, a, ref):
        d = a - ref
        d = (d + np.pi) % (2*np.pi) - np.pi
        return ref + d

    def _lpf_update(self, y_prev, x_now, tau, dt):
        alpha = dt / (tau + dt) if dt > 0.0 else 1.0
        return y_prev + alpha * (x_now - y_prev)

    def run(self, roll: float, pitch: float):
        t_now = self.clock.now()
        dt = (t_now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3

        meas = np.array([roll, pitch])

        # 첫 호출 시 현재 측정으로 초기화
        if not self.initialized:
            self.meas_filt = meas.copy()
            self.last_time = t_now
            self.last_error = self.desired_roll_pitch - self.meas_filt
            self.initialized = True

        # 언랩: 측정을 직전 필터값 주변의 연속각으로 맞춤
        meas = self._nearest_angle(meas, self.meas_filt)

        # 측정값 LPF
        self.meas_filt = self._lpf_update(self.meas_filt, meas, self.meas_tau, dt)

        # 에러 + 데드밴드
        error = self.desired_roll_pitch - self.meas_filt
        for i in range(2):
            if abs(error[i]) < self.deadband:
                error[i] = 0.0

        # 적분항 (클램프(조건부 적분: 출력이 바깥으로 포화되고 있고, 그 방향으로 적분이 더 커지려는 경우 적분 정지))
        self.I_term += error * dt
        self.I_term = np.clip(self.I_term, -self.i_limit, self.i_limit)

        # 미분항(오차 시간미분) + LPF
        raw_D = (error - self.last_error) / dt
        self.d_filt = self._lpf_update(self.d_filt, raw_D, self.d_tau, dt)

        # PID 합성
        P = self.kp * error
        I = self.ki * self.I_term
        D = self.kd * self.d_filt
        u = P + I + D

        # 출력 제한 + 조건부 적분(앤티윈드업)
        u_clipped = np.clip(u, -self.out_limit, self.out_limit)
        for i in range(2):
            if u[i] != u_clipped[i]:  # 포화 시 적분 되돌림
                if u[i] > 0.0 and error[i] > 0.0:
                    self.I_term[i] -= error[i] * dt
                if u[i] < 0.0 and error[i] < 0.0:
                    self.I_term[i] -= error[i] * dt
        self.I_term = np.clip(self.I_term, -self.i_limit, self.i_limit)

        # 상태 갱신
        self.last_time = t_now
        self.last_error = error.copy()
        return u_clipped
