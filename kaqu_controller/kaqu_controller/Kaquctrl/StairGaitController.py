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
        
    def run(self, state, command):
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height

        new_foot_locations = state.foot_location.copy()
        
        # IMU 보정
        if self.use_imu:
            # IMU에서 받은 기울기 (라디안)
            roll = state.imu_roll
            pitch = state.imu_pitch
            
            # PID 컨트롤러를 이용해 roll/pitch 오차 보정
            # PIDController의 run 메서드는 오차를 받아 보정값을 반환합니다.
            # 이 보정값을 롤/피치 각도에 더하거나 빼는 방식으로 사용합니다.
            # (현재 코드에서는 corrections[0]이 roll, corrections[1]이 pitch를 의미)
            corrections = self.pid_controller.run(roll, pitch)
            # corrections = [roll, pitch]로 임시 사용 가능

            for leg_index in range(4):
                x = new_foot_locations[0, leg_index]
                y = new_foot_locations[1, leg_index]
                z = new_foot_locations[2, leg_index]

                # --- x, y, z 개별 보정 로직 ---
                
                # 피치(pitch)에 의한 x 보정 (z 위치 기준)
                dx_pitch = -z * np.tan(pitch)
                
                # 롤(roll)에 의한 y 보정 (z 위치 기준)
                dy_roll = z * np.tan(roll)
                
                # 피치(pitch)에 의한 z 보정 (x 위치 기준)
                dz_pitch = -x * np.tan(pitch)
                
                # 롤(roll)에 의한 z 보정 (y 위치 기준)
                dz_roll = y * np.tan(roll)
                
                # --- 최종 보정값 적용 ---
                
                # x, y, z 좌표에 각각의 보정값 적용
                new_foot_locations[0, leg_index] += dx_pitch
                new_foot_locations[1, leg_index] += dy_roll
                new_foot_locations[2, leg_index] += (dz_pitch + dz_roll)

        return new_foot_locations

        # IMU 기반 회전 보정 적용
        
