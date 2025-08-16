from kaqu_controller.Kaquctrl.TrotGaitController import TrotGaitController, TrotSwingController, TrotStanceController
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
        z_error_constant = 0.5*4 # This constant determines how fast we move toward the goal in the z direction

        self.swingController = TrotSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                    self.phase_length, z_leg_lift, self.default_stance)
        
        self.stanceController = TrotStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                      self.time_step, z_error_constant)
        
        # PID 컨트롤러 객체 생성
        self.pid_controller = PID_controller(0.5, 0.02, 0.002)  # 이 숫자들은 임시 값 (kp, ki, kd)
        self.pid_controller.reset()  # 내부 변수들 초기화

        # PID 보정 게인 조정 
    def run(self, state, command):  # 외부에서 이 컨트롤러를 사용할 때 호출하는 최종 메서드: 한 틱씩 step()을 돌려 발 위치 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height  # 현재 높이를 state에 넣기

        new_foot_locations = state.foot_location.copy()
        # # imu compensation IMU 보정
        if self.use_imu: 
            # IMU에서 받은 기울기 (deg)
            roll = state.imu_roll
            pitch = state.imu_pitch
            # PID 컨트롤러를 이용해 roll/pitch 오차 보정
            corrections = [roll, pitch]
            # corrections = self.pid_controller.run(roll, pitch)
            # corrections *= -1
            for leg_index in range(4):
                # x = new_foot_locations[0, leg_index]
                # y = new_foot_locations[1, leg_index]
                # z = new_foot_locations[2, leg_index]

                new_z = command.robot_height*np.cos(corrections[1])*np.cos(corrections[0])
                dz = -1*(command.robot_height-new_z)
                new_foot_locations[2, leg_index] += dz

                dx = (-1*new_foot_locations[2,leg_index])*np.tan(corrections[1])
                dy = (1*new_foot_locations[2,leg_index])*np.tan(corrections[0])

                new_foot_locations[0, leg_index] += dx  
                new_foot_locations[1, leg_index] += dy

                
        return new_foot_locations

        # IMU 기반 회전 보정 적용
        
