### Written by TEAM4

from enum import Enum
import numpy as np


class BehaviorState(Enum):
    START = 0
    TROT = 1
    REST = 2
    STAIR = 3

class RobotState(object):
    def __init__(self, default_height):
        self.velocity = [0.0, 0.0]
        self.yaw_rate = 0.0
        self.robot_height = -default_height
        self.imu_roll = 0.0
        self.imu_pitch = 0.0

        self.foot_location = np.zeros((3,4))
        self.body_local_position = np.array([0., 0., 0.])
        self.body_local_orientation = np.array([0., 0., 0.])

        self.ticks = 0

        self.behavior_state = BehaviorState.REST

class RobotCommand(object):
    def __init__(self, default_height):
        self.trot_event = False
        self.rest_event = False
        self.start_event = False
        self.stair_event = False
        
        self.velocity = [0.0, 0.0]
        self.yaw_rate = 0.0
        self.robot_height = -default_height
        self.max_height = -205
        self.min_height = -110

class LegParameters(object):
    def __init__(self):
        self.pose = self.Leg_Pose()
        self.gait = self.Trot_Gait_Param()
        self.stair = self.Stair_Gait_Param()
        self.physical = self.Physical_Params()
    
    class Leg_Pose():
        def_stance = np.array([[0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]])
        initial_pose = np.array([[0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]])
    
    class Trot_Gait_Param():
        def __init__(self):
            self.cycle_time = None
            unit_time = 0.1
            self.stance_time = unit_time * 1.8
            self.swing_time = unit_time * 2.4
            self.time_step = 0.02
            self.max_x_vel = 30
            self.max_y_vel = 10
            self.max_yaw_rate = 0.3
            
            self.z_leg_lift = 40
            self.body_shift_y = 0.025 # Crawl 보행의 좌우 이동 폭 (미터 단위)

    class Stair_Gait_Param():
        def __init__(self):
            self.cycle_time = None
            unit_time = 0.1
            self.stance_time = unit_time * 2.0
            self.swing_time = unit_time * 2.6
            self.time_step = 0.02
            self.max_x_vel = 20
            self.max_y_vel = 8
            self.max_yaw_rate = 0.2
            self.z_leg_lift = 80
            self.robot_height = 180 

    class Physical_Params():
        l1 = 0.0
        l2 = 42.4
        l3 = 101.0
        l4 = 108.9