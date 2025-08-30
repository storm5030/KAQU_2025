from kaqu_controller.Kaquctrl.TrotGaitController import TrotGaitController, TrotSwingController, TrotStanceController
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters
from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz, rotz
from kaqu_controller.Kaquctrl.LowPassFilter import LowPassEMA, lpf_roll_pitch
from kaqu_controller.Kaquctrl.ContactUtils import simple_contact_state
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
        self.pid_controller = PID_controller(0.5, 0.02, 0.002)  # 이 숫자들은 임시 값 (kp, ki, kd)
        self.pid_controller.reset()  # 내부 변수들 초기화

        # IMU센서 보정용 필터 세팅
        self.lpf_r = LowPassEMA(init=0.0)
        self.lpf_p = LowPassEMA(init=0.0)

        # PID 보정 게인 조정 
    def run(self, state, command):
        # 1) 기본 보행 궤적 (base_link 좌표계)
        state.foot_location = self.step(state, command)   # shape: (3,4)
        state.robot_height = command.robot_height

        new_foot_locations = state.foot_location.copy()   # (3,4)
    
        body = [210.0, 140.5] # 길이와 너비
        legs = [0.0, 42.4, 101.0, 108.9] # 다리 링크 길이가 아니라 계산에 필요한 길이임
        x_shift_front = 0 #42
        x_shift_back = 0 #-15

        self.inverse_kinematics = InverseKinematics(body, legs,x_shift_front,x_shift_back)

        if self.use_imu:
            roll, pitch = lpf_roll_pitch(state, self.lpf_r, self.lpf_p)

            local_positions = self.inverse_kinematics.get_local_positions_from_world(new_foot_locations, dx=0, dy=0, dz=0, roll=0, pitch=0, yaw=0).T

            # 회전 보정: 몸체 기울기만큼 반대로 회전시켜 지면 평행 보행 평면 유지
            # rotxyz(rx, ry, rz)는 (X→Y→Z) 순 회전을 가정
            R_comp = rotxyz(-roll, -pitch, 0.0)   # 3x3

            # (3x3)@(3x4) = (3x4), 각 다리 열벡터에 일괄 적용
            local_positions = (R_comp @ local_positions).T

            new_foot_locations = self.inverse_kinematics.get_world_positions_from_leg(local_positions, dx=0, dy=0, dz=0, roll=0, pitch=0, yaw=0)
        
        # 임시 접촉 판단 확인 코드 (토크 센서)
        # print(state.tau_vector)
        # contact_states = simple_contact_state(state.tau_vector)
        # print(self.contacts(state.ticks))
        # print(contact_states)
        # print()
        # 콘택트 센서 출력
        # print(self.contacts(state.ticks))
        # print(state.contact_flags)

        # 최종 출력은 base_link 좌표계 유지
        return new_foot_locations


        # IMU 기반 회전 보정 적용
        
class StairSwingController(TrotSwingController):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        super().__init__(stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance)

        self.T_lift = 0.35 # 발을 드는 시간 (총 phase 1.0 기준)
        self.T_forward = 0.7 # 발을 전진하는 시간
        self.T_fall = 1.0 # 발을 딛는 시간 (사용하지 않음)

    def raibert_touchdown_location(self, leg_index, command, swing_phase):
        # 발을 내릴 때 진행 방향으로 뻗기
        # 발이 이번 swing에서 최종적으로 도달해야 하는 위치를 리턴함. (속도 계산 X)
        delta_pos_2d = np.array(command.velocity) * self.phase_length * self.time_step
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

        theta = self.stance_ticks * self.time_step * command.yaw_rate * 2
        rotation = rotz(theta)

        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def swing_height(self, swing_phase):  # 발 높이
        # T_lift까지 발 들기
        if swing_phase < self.T_lift:
            swing_height_ = swing_phase / self.T_lift * self.z_leg_lift
        # T_forward에는 발 진행
        elif swing_phase < self.T_forward:
            swing_height_ = self.z_leg_lift
        # 이후 내려놓기
        else:
            swing_height_ = self.z_leg_lift * ((1 - swing_phase) / (1 - self.T_forward))
        # print(swing_phase, ": ", swing_height_, "\n")
        return swing_height_
    
    def next_foot_location(self, swing_prop, leg_index, state, command):  # leg_index = 몇 번째 다리인
        #assert 0 <= swing_prop <= 1  # 진행률
        swing_prop += (1/self.swing_ticks) #0 ~ 0.9 -> 0.1~1.0
        foot_location = state.foot_location[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, swing_prop)

        # 마지막 틱 예외처리
        if swing_prop >= 1.0:
            new_position = touchdown_location * np.array([1, 1, 0]) + np.array([0, 0, command.robot_height])
            #[x,y,z]
            return new_position
        
        # 여기에서 발의 x, y방향 속도 계산
        back_gain = 0.0 # 발을 들 때 역방향 진행 속도 비율
       
        # 일단 발을 뒤로 빼지 않고, 사각형 모양으로 발 궤적을 그림 테스트
        # 발이 걸릴 경우 뒤로 빼는 방식 고려
        if swing_prop < self.T_lift:
            velocity = np.array([0.0, 0.0, 0.0])
            # time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)  # 남은 시간 계산
            # velocity = -back_gain * (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])
        elif swing_prop < self.T_forward:
            time_left = self.time_step * self.swing_ticks * (self.T_forward - swing_prop)  # 남은 시간 계산
            velocity = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])
        else:
            velocity = np.array([0.0, 0.0, 0.0])

        delta_foot_location = velocity * self.time_step  # 이번 스텝에서 foot_location이 얼마나 이동할지
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])  # 스윙 중 추가로 들어 올리기 + 로봇 몸체가 원하는 기본 높이\
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
