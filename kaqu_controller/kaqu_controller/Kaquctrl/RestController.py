#restgait

import rclpy
import numpy as np
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz
from kaqu_controller.KaquCmdManager.KaquParams import RobotState, RobotCommand

class RestController:
    def __init__(self, default_stance):
        self.def_stance = default_stance

        # Initialize the PID controller with default gains
        self.pid_controller = PID_controller(0.0, 0.0, 0.0)
        self.use_imu = False  # IMU compensation is initially disabled
        self.use_button = True  # Allows toggling IMU usage with a button
        self.pid_controller.reset()  # Reset the PID controller

    def updateStateCommand(self, msg, state, command):
        # Update local body position based on joystick input
        # state.body_local_position[0] = msg.axes[3] * -10
        # state.body_local_position[1] = msg.axes[4] * 10
        # state.body_local_position[2] = msg.axes[7] * 10
        # 시험용

        # robot_height는 음수값이므로 -5를 곱함, min,max도 마찬가지로 음수이므로 부등호 반대
        # 나중에 robot_height를 양수로 바꾸는 작업 필요할듯?
        command.robot_height = command.robot_height +msg.axes[7] * -5
        if (command.robot_height < command.max_height):
            command.robot_height = command.max_height
        if (command.robot_height > command.min_height):
            command.robot_height = command.min_height

        # Update local body orientation based on joystick input
        state.body_local_orientation[0] = msg.axes[0] * 0.2  # Roll
        state.body_local_orientation[1] = msg.axes[1] * 0.25  #pitch

        # IMU toggle logic
        if self.use_button:
            if msg.buttons[6]:  # Button press toggles IMU usage
                self.use_imu = not self.use_imu
                self.use_button = False
                print(f"RESTController - Use rp compensation: {self.use_imu}")
        else:
            if not msg.buttons[7]:  # Reset button state when released
                self.use_button = True

    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        # Copy the default stance and adjust for robot height
        temp = self.default_stance
        temp[2] = [command.robot_height] * 4

        roll_compensation = state.body_local_orientation[0]
        pitch_compensation = state.body_local_orientation[1]

        rot = rotxyz(roll_compensation, pitch_compensation, 0)
        temp = np.matmul(rot, temp)

        if self.use_imu:
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]
            rot = rotxyz(roll_compensation, pitch_compensation, 0)
            temp = np.matmul(rot, temp)

        return temp

    def run(self, state, command):
        # Update foot locations for the current step
        state.foot_location = self.step(state, command)
        return state.foot_location