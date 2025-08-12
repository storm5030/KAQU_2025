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
    def run(self, state, command):  # 외부에서 이 컨트롤러를 사용할 때 호출하는 최종 메서드: 한 틱씩 step()을 돌려 발 위치 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height  # 현재 높이를 state에 넣기

        new_foot_locations = state.foot_location.copy()
        # # imu compensation IMU 보정
        if self.use_imu: 
            # IMU에서 받은 기울기 (deg)
            roll = np.deg2rad(state.imu_roll)
            pitch = np.deg2rad(state.imu_pitch)
            # PID 컨트롤러를 이용해 roll/pitch 오차 보정
            corrections = [roll, pitch]
            for leg_index in range(4):
                x = new_foot_locations[0, leg_index]
                y = new_foot_locations[1, leg_index]
                z = new_foot_locations[2, leg_index]

                dx = z * np.tan(pitch)    
                # pitch에 의한 z 보정 (x 위치 기준)
                dz_pitch = x * np.tan(corrections[1])

                dy = -z * np.tan(roll)
                # roll에 의한 z 보정 (y 위치 기준)  
                dz_roll = -y * np.tan(corrections[0])

                # 최종 보정값
                dz = dz_pitch + dz_roll
                
                # 좌표에 보정 적용
                new_foot_locations[0, leg_index] += dx
                new_foot_locations[1, leg_index] += dy
                new_foot_locations[2, leg_index] += dz

        return new_foot_locations

        # IMU 기반 회전 보정 적용
        
