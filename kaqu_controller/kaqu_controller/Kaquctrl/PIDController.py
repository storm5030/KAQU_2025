#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType

class PID_controller(object):
    """
    Roll/Pitch 안정화용 PID
    - 입력 단위: 라디안 (roll, pitch)
    - 출력 단위: 라디안 (보정 각도)
    - 내부에 측정값 LPF, 미분항 LPF, 적분 앤티윈드업, 출력 제한, 데드밴드 포함
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
        self.deadband = np.deg2rad(abs(deadband_deg))  # 라디안으로 저장

        self.desired_roll_pitch = np.array([0.0, 0.0])  # 목표값(기본 0)
        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        self.reset()

    def reset(self):
        self.last_time = self.clock.now()
        self.last_error = np.array([0.0, 0.0])

        # LPF 상태
        self.meas_filt = np.array([0.0, 0.0])   # roll, pitch 필터링 값
        self.d_filt = np.array([0.0, 0.0])      # 미분항 필터링 값

        # 적분항
        self.I_term = np.array([0.0, 0.0])

    def desired_RP_angles(self, des_roll: float, des_pitch: float):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])

    def _lpf_update(self, y_prev: np.ndarray, x_now: np.ndarray, tau: float, dt: float):
        # 1차 저역통과: y += alpha*(x - y), alpha=dt/(tau+dt)
        alpha = dt / (tau + dt) if dt > 0.0 else 1.0
        return y_prev + alpha * (x_now - y_prev)

    def run(self, roll: float, pitch: float):
        # 시간계산
        t_now = self.clock.now()
        dt = (t_now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3  # 비정상 시 최소 dt

        # 1) 측정값 저역통과
        meas = np.array([roll, pitch])
        self.meas_filt = self._lpf_update(self.meas_filt, meas, self.meas_tau, dt)

        # 2) 에러 계산 + 데드밴드
        error = self.desired_roll_pitch - self.meas_filt
        for i in range(2):
            if abs(error[i]) < self.deadband:
                error[i] = 0.0

        # 3) 적분항 업데이트 (+ 앤티윈드업용 클램프)
        self.I_term += error * dt
        self.I_term = np.clip(self.I_term, -self.i_limit, self.i_limit)

        # 4) 미분항 (오차의 시간미분) + 저역통과
        raw_D = (error - self.last_error) / dt
        self.d_filt = self._lpf_update(self.d_filt, raw_D, self.d_tau, dt)

        # 5) PID 합성
        P = self.kp * error
        I = self.ki * self.I_term
        D = self.kd * self.d_filt
        u = P + I + D

        # 6) 출력 제한 + 조건부 적분 보정
        u_clipped = np.clip(u, -self.out_limit, self.out_limit)
        for i in range(2):
            if u[i] != u_clipped[i]:  # 포화
                if u[i] > 0.0 and error[i] > 0.0:
                    self.I_term[i] -= error[i] * dt
                if u[i] < 0.0 and error[i] < 0.0:
                    self.I_term[i] -= error[i] * dt
        self.I_term = np.clip(self.I_term, -self.i_limit, self.i_limit)

        # 마무리
        self.last_time = t_now
        self.last_error = error.copy()
        return u_clipped
