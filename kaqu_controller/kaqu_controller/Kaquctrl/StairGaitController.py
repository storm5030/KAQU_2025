from kaqu_controller.Kaquctrl.TrotGaitController import TrotGaitController, TrotSwingController, TrotStanceController
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz, rotz
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

        self.swingController = StairSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                    self.phase_length, z_leg_lift, self.default_stance)
        
        self.stanceController = TrotStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                      self.time_step, z_error_constant)
        
        # PID 컨트롤러 객체 생성
        self.pid_controller = PID_controller(0.12, 0.30, 0.01, 0.15, 0.05, 0.20, 0.10, 0.5)  # 이 숫자들은 임시 값 (kp, ki, kd, meas_tau, d_tau, out_limit, i_limit, deadband_deg)
        self.pid_controller.reset()  # 내부 변수들 초기화

        # PID 보정 게인 조정 
    def run(self, state, command):  # 외부에서 이 컨트롤러를 사용할 때 호출하는 최종 메서드: 한 틱씩 step()을 돌려 발 위치 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height  # 현재 높이를 state에 넣기

        new_foot_locations = state.foot_location.copy()
        # # imu compensation IMU 보정
        if self.use_imu:
            roll = state.imu_roll
            pitch = state.imu_pitch

            # 가드: 큰 기울기/전복 시 보정 OFF + PID 리셋
            if self.imu_gate and (abs(roll) > self.th_off or abs(pitch) > self.th_off or getattr(state, "is_inverted", False)):
                self.imu_gate = False
                self.pid_controller.reset()
            elif (not self.imu_gate) and (abs(roll) < self.th_on and abs(pitch) < self.th_on and not getattr(state, "is_inverted", False)):
                self.imu_gate = True

            if self.imu_gate:
                corrections = self.pid_controller.run(roll, pitch)  # [roll_corr, pitch_corr]
                t_roll  = np.tan(corrections[0])
                t_pitch = np.tan(corrections[1])
                for leg_index in range(4):
                    x = new_foot_locations[0, leg_index]
                    y = new_foot_locations[1, leg_index]
                    # z-only 보정 (계단에서 XY 드리프트 방지)
                    new_foot_locations[2, leg_index] += x * t_pitch - y * t_roll

                
        return new_foot_locations

        
class StairSwingController(TrotSwingController):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        if swing_phase < 0.5:
            return self.default_stance[:, leg_index]
        else:
            delta_pos_2d = 2 * np.array(command.velocity) * self.phase_length * self.time_step
            delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

            theta = self.stance_ticks * self.time_step * command.yaw_rate * 2
            rotation = rotz(theta)

            return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def swing_height(self, swing_phase):  # 0.5까지는 들고 이후는 내려놓기
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase-0.5) / 0.5)
        print(swing_phase, ": ", swing_height_, "\n")
        return swing_height_
    
    def next_foot_location(self, swing_prop, leg_index, state, command):  # leg_index = 몇 번째 다리인
        #assert 0 <= swing_prop <= 1  # 진행률
        swing_prop += (1/self.swing_ticks) #0 ~ 0.9 -> 0.1~1.0
        foot_location = state.foot_location[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, swing_prop)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)  # 남은 시간 계산
        # 마지막 틱 예외처리
        if swing_prop >= 1.0:
            new_position = touchdown_location * np.array([1, 1, 0]) + np.array([0, 0, command.robot_height])
            #[x,y,z]
            return new_position
        
        velocity = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])

        delta_foot_location = velocity * self.time_step  # 이번 스텝에서 foot_location이 얼마나 이동할지
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])  # 스윙 중 추가로 들어 올리기 + 로봇 몸체가 원하는 기본 높이\
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
