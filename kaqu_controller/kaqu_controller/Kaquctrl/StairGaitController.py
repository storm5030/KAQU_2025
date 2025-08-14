# kaqu_controller/Kaquctrl/StairGaitController.py

from kaqu_controller.Kaquctrl.TrotGaitController import TrotGaitController
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters
import numpy as np


class StairTrotGaitController(TrotGaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        super().__init__(default_stance, stance_time, swing_time, time_step, use_imu)

        # 계단용 파라미터 반영
        leg = LegParameters()
        self.stair = leg.stair                      # 계단용 파라미터 묶음
        self.max_x_vel   = self.stair.max_x_vel
        self.max_y_vel   = self.stair.max_y_vel
        self.max_yaw_rate = self.stair.max_yaw_rate
        # (z_leg_lift는 상위 TrotGaitController에서 사용될 수 있음)

        # PID 컨트롤러 (IMU 보정용)
        self.pid_controller = PID_controller(kp=0.1, ki=0.0, kd=0.01)
        self.pid_controller.reset()

    def step(self, state, command):
        """
        부모(TrotGaitController)의 step()으로 기본 발 위치를 만든 뒤,
        IMU roll/pitch로 z 보정만 추가 적용해서 반환.
        """
        new_foot_locations = super().step(state, command)

        if not self.use_imu:
            return new_foot_locations

        # IMU 값(일반적으로 deg)을 PID로 한번 거쳐 보정 게인화
        compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)

        # run()이 [roll, pitch] 형태를 준다고 가정(단일 스칼라가 오면 안전하게 처리)
        if isinstance(compensation, (list, tuple)) and len(compensation) >= 2:
            roll_comp, pitch_comp = float(compensation[0]), float(compensation[1])
        else:
            roll_comp, pitch_comp = float(compensation), 0.0

        # deg -> rad (IMU가 deg면)
        roll_rad  = np.deg2rad(roll_comp)
        pitch_rad = np.deg2rad(pitch_comp)

        # 각 다리 z 보정: dz = (-y)*tan(roll) + (x)*tan(pitch)
        for leg_idx in range(4):
            x = new_foot_locations[0, leg_idx]
            y = new_foot_locations[1, leg_idx]

            dz = (-y * np.tan(roll_rad)) + (x * np.tan(pitch_rad))
            dz = np.clip(dz, -30.0, 30.0)  # 과한 보정 제한

            new_foot_locations[2, leg_idx] += dz

        return new_foot_locations