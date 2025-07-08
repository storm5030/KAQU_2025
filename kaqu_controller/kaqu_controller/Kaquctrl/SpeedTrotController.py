import rclpy
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
from kaqu_controller.Kaquctrl.GaitController import GaitController
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz, rotz
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters

class SpeedTrotGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        self.use_imu = use_imu
        self.use_button = True
        self.autoRest = True
        self.trotNeeded = True

        leg = LegParameters()
        
        contact_phases = np.array([
            [1, 0],  # 0: Leg swing
            [0, 1],  # 1: Moving stance forward
            [0, 1],
            [1, 0]
        ])

        z_error_constant = 0.5 * 4  # This constant determines how fast we move toward the goal in the z direction

        z_leg_lift = leg.gait.z_leg_lift
        # 부모 클래스 (GaitController) 호출
        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.max_x_vel = leg.gait.max_x_vel
        self.max_y_vel = leg.gait.max_y_vel
        self.max_yaw_rate = leg.gait.max_yaw_rate
        # swing이랑 stance에 정보주는거?
        self.swingController = SpeedTrotSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                    self.phase_length, z_leg_lift, self.default_stance)

        self.stanceController = SpeedTrotStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                      self.time_step, z_error_constant)

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.)  # 게인이 0이라 효력 X

    # 선속도와 각속도를 스케일링
    def updateStateCommand(self, msg, command):
        command.velocity[0] = msg.axes[4] * self.max_x_vel
        command.velocity[1] = msg.axes[3] * self.max_y_vel
        command.yaw_rate = msg.axes[6] * self.max_yaw_rate

        print(f"Velocity X: {command.velocity[0]}, Y: {command.velocity[1]}, Yaw Rate: {command.yaw_rate}")

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

            for leg_index in range(4):
                contact_mode = contact_modes[leg_index]
                if contact_mode == 1:  # 접지해 있다면 스탠스
                    new_location = self.stanceController.next_foot_location(leg_index, state, command)
                else:  # 아니라면 스윙 반환
                    swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                    new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)
                new_foot_locations[:, leg_index] = new_location
       

            # imu compensation IMU 보정
            if self.use_imu:
                compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                roll_compensation = -compensation[0]
                pitch_compensation = -compensation[1]

                rot = rotxyz(roll_compensation, pitch_compensation, 0)
                new_foot_locations = np.matmul(rot, new_foot_locations)

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
class SpeedTrotSwingController(object):
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
        swing_phase += 0.1
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command):  # leg_index = 몇 번째 다리인
        assert 0 <= swing_prop <= 1  # 진행률
        foot_location = state.foot_location[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)  # 남은 시간 계산

        velocity = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])

        delta_foot_location = velocity * self.time_step  # 이번 스텝에서 foot_location이 얼마나 이동할지
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])  # 스윙 중 추가로 들어 올리기 + 로봇 몸체가 원하는 기본 높이
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location

class SpeedTrotStanceController(object):
    def __init__(self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg_index, state, command):
        z = state.foot_location[2, leg_index]

        step_dist_x = command.velocity[0] * (float(self.phase_length) / self.swing_ticks)
        step_dist_y = command.velocity[1] * (float(self.phase_length) / self.swing_ticks)

        velocity = np.array([
            -(step_dist_x / 4) / (float(self.time_step) * self.stance_ticks),
            -(step_dist_y / 4) / (float(self.time_step) * self.stance_ticks),
            0
            #1.0 / self.z_error_constant * (state.robot_height - z)
        ])

        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.yaw_rate * self.time_step)
        return (delta_pos, delta_ori)

    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_location[:, leg_index]  # 현재 다리 끝
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos  # 이동해야 할 만큼 다리를 반대로 밀기
        return next_foot_location
  

# import rclpy
# import numpy as np
# from geometry_msgs.msg import Twist, TwistStamped
# from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
# from kaqu_controller.Kaquctrl.GaitController import GaitController
# from kaqu_controller.Kaquctrl.PIDController import PID_controller
# from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz, rotz
# from kaqu_controller.KaquCmdManager.KaquParams import LegParameters

# class SpeedTrotGaitController(GaitController):
#     def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
#         self.use_imu = use_imu
#         self.use_button = True
#         self.autoRest = True
#         self.trotNeeded = True

#         leg = LegParameters()
        
#         contact_phases = np.array([
#             [1, 0],  # 0: Leg swing
#             [0, 1],  # 1: Moving stance forward
#             [0, 1],
#             [1, 0]
#         ])

#         z_error_constant = 80  # z 보정 상수

#         z_leg_lift = leg.gait.z_leg_lift
#         super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

#         self.max_x_vel = leg.gait.max_x_vel
#         self.max_y_vel = leg.gait.max_y_vel
#         self.max_yaw_rate = leg.gait.max_yaw_rate

#         self.swingController = SpeedTrotSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
#                                                     self.phase_length, z_leg_lift, self.default_stance)

#         self.stanceController = SpeedTrotStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
#                                                       self.time_step, z_error_constant)

#         self.pid_controller = PID_controller(0.5, 0.1, 0.05)  # PID 게인 적용

#     def updateStateCommand(self, msg, command):
#         command.velocity[0] = msg.axes[4] * self.max_x_vel
#         command.velocity[1] = msg.axes[3] * self.max_y_vel
#         command.yaw_rate = msg.axes[6] * self.max_yaw_rate

#     def step(self, state, command):
#         if self.autoRest:  
#             if command.velocity[0] == 0 and command.velocity[1] == 0 and command.yaw_rate == 0:
#                 if state.ticks % (2 * self.phase_length) == 0:
#                     self.trotNeeded = False
#             else:
#                 self.trotNeeded = True

#         if self.trotNeeded:
#             contact_modes = self.contacts(state.ticks)
#             new_foot_locations = np.zeros((3, 4))

#             for leg_index in range(4):
#                 if contact_modes[leg_index] == 1:
#                     new_location = self.stanceController.next_foot_location(leg_index, state, command)
#                 else:
#                     swing_prop = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
#                     new_location = self.swingController.next_foot_location(swing_prop, leg_index, state, command)
#                 new_foot_locations[:, leg_index] = new_location

#             if self.use_imu:
#                 compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
#                 roll_comp = -compensation[0]
#                 pitch_comp = -compensation[1]

#                 rot = rotxyz(roll_comp, pitch_comp, 0)
#                 for i, contact in enumerate(contact_modes):
#                     if contact == 1:
#                         new_foot_locations[:, i] = np.matmul(rot, new_foot_locations[:, i])

#             state.ticks += 1
#             return new_foot_locations
#         else:
#             temp = self.default_stance.copy()
#             temp[2] = [command.robot_height] * 4
#             return temp
        
#     def run(self, state, command):
#         """ 한 틱씩 step()을 돌려 발 위치 업데이트 """
#         state.foot_location = self.step(state, command)
#         state.robot_height = command.robot_height  # 현재 높이를 state에 반영
#         return state.foot_location

# class SpeedTrotSwingController:
#     def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
#         self.stance_ticks = stance_ticks
#         self.swing_ticks = swing_ticks
#         self.time_step = time_step
#         self.phase_length = phase_length
#         self.z_leg_lift = z_leg_lift
#         self.default_stance = default_stance

#     def raibert_touchdown_location(self, leg_index, command):
#         delta_pos_2d = np.array(command.velocity) * self.phase_length * self.time_step
#         delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

#         theta = self.stance_ticks * self.time_step * command.yaw_rate
#         rotation = rotz(theta)

#         return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

#     def swing_height(self, swing_phase):
#         t = swing_phase
#         return (1 - t) * t * 4 * self.z_leg_lift  # 베지어 곡선 적용

#     def next_foot_location(self, swing_prop, leg_index, state, command):
#         assert 0 <= swing_prop <= 1
#         foot_location = state.foot_location[:, leg_index]
#         swing_height_ = self.swing_height(swing_prop)
#         touchdown_location = self.raibert_touchdown_location(leg_index, command)

#         time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)

#         velocity = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])

#         delta_foot_location = velocity * self.time_step
#         z_vector = np.array([0, 0, swing_height_ + command.robot_height])
#         return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location

# class SpeedTrotStanceController:
#     def __init__(self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant):
#         self.phase_length = phase_length
#         self.stance_ticks = stance_ticks
#         self.swing_ticks = swing_ticks
#         self.time_step = time_step
#         self.z_error_constant = z_error_constant

#     def position_delta(self, leg_index, state, command):
#         z = state.foot_location[2, leg_index]

#         step_dist_x = command.velocity[0] * (float(self.phase_length) / self.swing_ticks)
#         step_dist_y = command.velocity[1] * (float(self.phase_length) / self.swing_ticks)

#         velocity = np.array([
#             -step_dist_x / (float(self.time_step) * self.stance_ticks),
#             -step_dist_y / (float(self.time_step) * self.stance_ticks),
#             (state.robot_height - z) / self.z_error_constant
#         ])

#         delta_pos = velocity * self.time_step
#         delta_ori = rotz(-command.yaw_rate * self.time_step)
#         return delta_pos, delta_ori

#     def next_foot_location(self, leg_index, state, command):
#         foot_location = state.foot_location[:, leg_index]
#         delta_pos, delta_ori = self.position_delta(leg_index, state, command)
#         next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
#         return next_foot_location