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
        # (Trot Gait의 autoRest 기능 등은 Crawl 로직에서는 사용되지 않으므로 일부 변수 선언을 제거하거나 수정합니다.)
        self.use_imu = use_imu
        self.crawlNeeded = True # trotNeeded -> crawlNeeded
        leg = LegParameters()
        
        # Crawl Gait의 8단계 접촉 시퀀스
        contact_phases = np.array([[1, 0, 1, 1, 1, 1, 1, 1],
                                   [1, 1, 1, 0, 1, 1, 1, 1],
                                   [1, 1, 1, 1, 1, 0, 1, 1],
                                   [1, 1, 1, 1, 1, 1, 1, 0]])
        
        # Crawl 레퍼런스에 맞는 파라미터 값으로 수정
        z_error_constant = 0.02
        z_leg_lift = leg.gait.z_leg_lift

        # *** 수정된 부분 1: body_shift_y 변수를 사용하기 전에 선언 ***
        self.body_shift_y = leg.gait.body_shift_y

        # 부모 클래스 (GaitController) 호출
        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        # *** 수정된 부분 2: Crawl 레퍼런스에 맞는 최대 속도 값으로 수정 ***
        self.max_x_vel = 0.003
        self.max_y_vel = 0.006
        self.max_yaw_rate = 0.15
        
        # Crawl 보행에 필요한 first_cycle 플래그 선언
        self.first_cycle = True
        
        # 하위 컨트롤러 초기화 (이제 self.body_shift_y가 존재하므로 정상 작동)
        self.swingController = CrawlSwingController(self.stance_ticks,
            self.swing_ticks, self.time_step, self.phase_length, z_leg_lift,
            self.default_stance, self.body_shift_y)
        self.stanceController = CrawlStanceController(self.phase_length,
            self.stance_ticks, self.swing_ticks, self.time_step,
            z_error_constant, self.body_shift_y)

        # PID 컨트롤러는 Crawl 레퍼런스에 없으므로 비활성화 또는 초기화만 진행
        self.pid_controller = PID_controller(0., 0., 0.)

    # 선속도와 각속도를 스케일링
    def updateStateCommand(self, msg, command):
        command.velocity[0] = msg.axes[4] * self.max_x_vel
        command.velocity[1] = msg.axes[3] * self.max_y_vel
        command.yaw_rate = msg.axes[6] * self.max_yaw_rate

        print(f"Velocity X: {command.velocity[0]}, Y: {command.velocity[1]}, Yaw Rate: {command.yaw_rate}")

    def step(self, state, command):
        # 1. Trot에 있던 자동 휴식 로직을 여기에 추가합니다.
        #    Crawl에 맞게 trotNeeded -> crawlNeeded 변수를 사용하도록 수정합니다.
        if command.velocity[0] == 0 and command.velocity[1] == 0 and command.yaw_rate == 0:
            if state.ticks % (2 * self.phase_length) == 0:
                self.crawlNeeded = False
        else:
            self.crawlNeeded = True

        # 2. crawlNeeded 플래그 값에 따라 분기합니다.
        if self.crawlNeeded:  # 움직여야 할 때만 Crawl 로직 실행
            contact_modes = self.contacts(state.ticks)
            new_foot_locations = np.zeros((3,4))

            phase_index = self.phase_index(state.ticks)
            for leg_index in range(4):
                contact_mode = contact_modes[leg_index]
                if contact_mode == 1:
                    if phase_index in (0,4) :
                        move_sideways = True
                        if phase_index == 0:
                            move_left = True
                        else:
                            move_left = False
                    else:
                        move_sideways = False
                        move_left = False

                    new_location = self.stanceController.next_foot_location(leg_index,
                        state, command, self.first_cycle, move_sideways, move_left)
                else:
                    swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                    if phase_index in (1,3):
                        shifted_left = True
                    else:
                        shifted_left = False

                    new_location = self.swingController.next_foot_location(swing_proportion,
                        leg_index, state, command, shifted_left)

                new_foot_locations[:, leg_index] = new_location
            
            state.ticks += 1 # state.ticks 업데이트를 step 안으로 이동
            return new_foot_locations
        
        else:  # 움직일 필요가 없을 때 (자동 휴식)
            temp = self.default_stance.copy()
            temp[2] = [command.robot_height] * 4
            return temp

    
    def run(self, state, command):  # 외부에서 이 컨트롤러를 사용할 때 호출하는 최종 메서드: 한 틱씩 step()을 돌려 발 위치 업데이트
        state.foot_location = self.step(state, command)
        state.ticks += 1
        state.robot_height = command.robot_height  # 현재 높이를 state에 넣기
        
        if self.phase_index(state.ticks) > 0 and self.first_cycle:
            self.first_cycle = False

        return state.foot_location 

# swing인 발의 위치 계산
class CrawlSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length,
                 z_leg_lift, default_stance, body_shift_y):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance
        self.body_shift_y = body_shift_y

    def raibert_touchdown_location(self, leg_index, command, shifted_left):
        # Crawl 로직으로 교체 (몸통 좌우 이동 보정 추가)
        
        delta_pos_2d = np.array(command.velocity) * self.phase_length * self.time_step
        
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0]) * 1000.0
        theta = self.stance_ticks * self.time_step * command.yaw_rate
        rotation = rotz(theta)
        shift_correction = np.array([0.,0.,0.])
        if shifted_left:
            shift_correction[1] = -self.body_shift_y
        else:
            shift_correction[1] = self.body_shift_y
        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos + shift_correction
    
    def swing_height(self, swing_phase):

        swing_phase += 0.1
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command, shifted_left):

        assert swing_prop >= 0 and swing_prop <= 1
        foot_location = state.foot_location[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, shifted_left)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        if time_left == 0:
            return touchdown_location

        velocity = (touchdown_location - foot_location) / float(time_left) *\
             np.array([1, 1, 0])

        delta_foot_location = velocity * self.time_step
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])

        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location

class CrawlStanceController(object):
    def __init__(self,phase_length, stance_ticks, swing_ticks, time_step,
                 z_error_constant,body_shift_y):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant
        self.body_shift_y = body_shift_y

    def position_delta(self, leg_index, state, command, first_cycle,
                       move_sideways, move_left):
        z = state.foot_location[2, leg_index]

        # SpeedTrot과 동일하게 step_dist_x와 step_dist_y를 모두 계산하도록 추가
        step_dist_x = command.velocity[0] * (float(self.phase_length) / self.swing_ticks)
        step_dist_y = command.velocity[1] * (float(self.phase_length) / self.swing_ticks)

        if first_cycle:
            shift_factor = 1
        else:
            shift_factor = 2

        side_vel = 0.0
        if move_sideways:
            if move_left:
                side_vel = -(self.body_shift_y*shift_factor)/(float(self.time_step)*self.stance_ticks)
            else:
                side_vel = (self.body_shift_y*shift_factor)/(float(self.time_step)*self.stance_ticks)

        # SpeedTrot 레퍼런스에서 Z축 속도를 0으로 설정한 것을 반영하여 Z축 보정 비활성화
        velocity = np.array([-(step_dist_x)/(float(self.time_step)*self.stance_ticks), 
                             side_vel, 
                             0
                             # 1.0 / self.z_error_constant * (state.robot_height - z) # SpeedTrot처럼 Z축 보정 비활성화
                            ])

        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.yaw_rate * self.time_step)
        return (delta_pos, delta_ori)

    def next_foot_location(self, leg_index, state, command, first_cycle, move_sideways, move_left):
        foot_location = state.foot_location[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command,
            first_cycle, move_sideways, move_left)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
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