import rclpy
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
from kaqu_controller.Kaquctrl.GaitController import GaitController
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz, rotz
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters

class TrotGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        #  77.5  77.5  -77.5 -77.5
        #-91.45 91.45 -91.45 91.45
        #     0     0      0     0
        # stance_time: unit_time*3 = 0.3, swing_time: unit_time*7 = 0.7, time_step: 0.1, imu
        self.use_imu = use_imu
        self.pid_controller = PID_controller(kp=1.0, ki=0.1, kd=0.05)  # PID 컨트롤러 인스턴스 생성
        self.use_button = True
        self.autoRest = True
        self.trotNeeded = True

        leg = LegParameters()
        #test
        
        contact_phases = np.array([
            [1, 1, 1, 0],  # 0: Leg swing
            [1, 0, 1, 1],  # 1: Moving stance forward
            [1, 0, 1, 1],
            [1, 1, 1, 0]
        ])
        # contact_phases = np.array([
        #     [1, 1, 0, 1],  # 0: Leg swing
        #     [0, 1, 1, 1],  # 1: Moving stance forward
        #     [0, 1, 1, 1],
        #     [1, 1, 0, 1]
        # ])


        z_error_constant = 0.5*4#0.5 * 4  # This constant determines how fast we move toward the goal in the z direction

        z_leg_lift = leg.gait.z_leg_lift 
        # from LegParameters.Trot_Gait_Params, 40

        # 부모 클래스 (GaitController) 호출
        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.max_x_vel = leg.gait.max_x_vel #20
        self.max_y_vel = leg.gait.max_y_vel #10
        self.max_yaw_rate = leg.gait.max_yaw_rate #0.1
        # swing이랑 stance에 정보주는거?
        self.swingController = TrotSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                    self.phase_length, z_leg_lift, self.default_stance)
        # stance_ticks: 3 swing_ticks: 7 time_step 0.1 phase_length: [3,7,3,7] = 20, 40,
        # default_stance
        #  77.5  77.5  -77.5 -77.5
        #-91.45 91.45 -91.45 91.45
        #     0     0      0     0
        self.stanceController = TrotStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                      self.time_step, z_error_constant)
        # stance_ticks: 3 swing_ticks: 7 time_step 0.1 phase_length: 20, z_error_constant: 1,
        # default_stance
        #  77.5  77.5  -77.5 -77.5
        #-91.45 91.45 -91.45 91.45
        #     0     0      0     0

        # TODO : 게인값 조율

    # 선속도와 각속도를 스케일링
    def updateStateCommand(self, msg, state, command):
        command.velocity[0] = msg.axes[4] * self.max_x_vel   # 앞뒤 움직임
        command.velocity[1] = msg.axes[3] * self.max_y_vel #조이스틱 오른쪽: -1 / 왼쪽: +1 /// y축은 왼쪽 기준
        command.yaw_rate = msg.axes[6] * self.max_yaw_rate

        # print(f"Velocity X: {command.velocity[0]}, Y: {command.velocity[1]}, Yaw Rate: {command.yaw_rate}")

    def step(self, state, command):
        if self.autoRest:  # 멈춰있으면 자동으로 휴식 자세
            if command.velocity[0] == 0 and command.velocity[1] == 0 and command.yaw_rate == 0:
                if state.ticks % (2 * self.phase_length) == 0:
                    self.trotNeeded = False
            else:
                self.trotNeeded = True

        print(f"Trot Needed: {self.trotNeeded}, Velocity: {command.velocity}, Yaw Rate: {command.yaw_rate}")

        if self.trotNeeded:  # 움직이고 있으면
            contact_modes = self.contacts(state.ticks)  # 접지 배열 가져오기
            new_foot_locations = np.zeros((3, 4))

    
             # IMU 보정치를 먼저 계산합니다.
            z_offsets = np.zeros(4)
            if self.use_imu:
                roll_deg = state.imu_roll
                pitch_deg = state.imu_pitch
                corrections = self.pid_controller.run(roll_deg, pitch_deg)
                for i in range(4):
                    # 새 위치가 아닌 현재 발 위치(state.foot_location)를 기준으로 오프셋을 계산합니다.
                    x = state.foot_location[0, i]
                    y = state.foot_location[1, i]
                    dz_pitch = -x * np.tan(np.radians(pitch_deg)) + corrections[1]
                    dz_roll = -y * np.tan(np.radians(roll_deg)) + corrections[0]
                    z_offsets[i] = dz_pitch + dz_roll

            for leg_index in range(4):
                contact_mode = contact_modes[leg_index]
                z_offset = z_offsets[leg_index] # 해당 다리의 z 오프셋

                if contact_mode == 1:
                    # StanceController 호출 시 계산된 z_offset을 전달합니다.
                    new_location = self.stanceController.next_foot_location(leg_index, state, command, z_offset)
                else:
                    swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                    # (추가 개선) SwingController에도 z_offset을 전달하여 착지 지점 높이를 보정할 수 있습니다.
                    new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)
                new_foot_locations[:, leg_index] = new_location

            state.ticks += 1
            return new_foot_locations

        else:  # 안 움직이고 있으면
            temp = self.default_stance.copy()
            temp[2] = [command.robot_height] * 4   

            return temp

    def run(self, state, command):  # 외부에서 이 컨트롤러를 사용할 때 호출하는 최종 메서드: 한 틱씩 step()을 돌려 발 위치 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height  # 현재 높이를 state에 넣기
        return state.foot_location

# swing인 발의 위치 계산
class TrotSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance

    def raibert_touchdown_location(self, leg_index, command):
        delta_pos_2d = np.array(command.velocity) * self.phase_length * self.time_step
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

        theta = self.stance_ticks * self.time_step * command.yaw_rate
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
        touchdown_location = self.raibert_touchdown_location(leg_index, command)

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

class TrotStanceController(object):
    def __init__(self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg_index, state, command, z_offset=0.0):
        z = state.foot_location[2, leg_index]
        #z = command.robot_height

        # 목표 높이에 IMU 보정치를 반영합니다.
        target_z = state.robot_height + z_offset

        step_dist_x = command.velocity[0] * (float(self.phase_length) / self.swing_ticks)
        step_dist_y = command.velocity[1] * (float(self.phase_length) / self.swing_ticks)

        velocity = np.array([
            -(step_dist_x / 4) / (float(self.time_step) * self.stance_ticks),
            -(step_dist_y / 4) / (float(self.time_step) * self.stance_ticks),
            # 수정된 목표 높이(target_z)를 사용해 오차를 계산합니다.
            1.0 / self.z_error_constant * (target_z - z)
        ])

        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.yaw_rate * self.time_step)
        return (delta_pos, delta_ori)

    def next_foot_location(self, leg_index, state, command, z_offset=0.0):
        foot_location = state.foot_location[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command, z_offset) # z_offset 전달
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location
