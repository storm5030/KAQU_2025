from kaqu_controller.Kaquctrl.TrotGaitController import TrotGaitController
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters
import numpy as np


class StairTrotGaitController(TrotGaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        super().__init__(default_stance, stance_time, swing_time, time_step, use_imu)

        # 발 높이 조정
        leg = LegParameters()
        z_leg_lift = leg.stair.z_leg_lift
        self.max_x_vel = leg.stair.max_x_vel 
        self.max_y_vel = leg.stair.max_y_vel 
        self.max_yaw_rate = leg.stair.max_yaw_rate
        
        # PID 컨트롤러 객체 생성
        self.pid_controller = PID_controller(0.1, 0.0, 0.01)  # 이 숫자들은 임시 값 (kp, ki, kd)
        self.pid_controller.reset()  # 내부 변수들 초기화

        # PID 보정 게인 조정 
    def step(self, state, command):
        new_foot_locations = super().step(state, command)  # 기존 보행 발 위치 계산

        if self.use_imu:
            # 1. 현재 기울기 정보 (roll, pitch)
            roll = state.imu_roll
            pitch = state.imu_pitch

            # 2. PID 보정값 계산
            compensation = self.pid_controller.run(roll, pitch)
            roll_comp = -compensation[0]  # 음수로 반영
            pitch_comp = -compensation[1]

            # 3. 각 다리에 보정값 적용
            for leg_index in range(4):
                x = new_foot_locations[0, leg_index]
                y = new_foot_locations[1, leg_index]

                # pitch 보정 → z 보정값
                dz_pitch = -x * np.tan(pitch_comp)

                # roll 보정 → z 보정값
                dz_roll = -y * np.tan(roll_comp)

                # 최종 보정값
                dz = dz_pitch + dz_roll
                dz = max(min(dz, 30.0), -30.0)  # 클리핑

                # z값 보정 적용
                new_foot_locations[2, leg_index] += dz

        return new_foot_locations

        # IMU 기반 회전 보정 적용
        
