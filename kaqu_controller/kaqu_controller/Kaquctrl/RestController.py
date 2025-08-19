#restgait

import rclpy
import numpy as np
from kaqu_controller.Kaquctrl.PIDController import PID_controller
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz
from kaqu_controller.KaquCmdManager.KaquParams import RobotState, RobotCommand

class RestController:
    def __init__(self, default_stance):
        self.def_stance = default_stance

        # 언랩/LPF/앤티윈드업 포함 PID (Rest는 출력 제한을 조금 더 타이트하게)
        self.pid_controller = PID_controller(
            kp=0.12, ki=0.30, kd=0.01,
            meas_tau=0.15, d_tau=0.05,
            out_limit=0.15,  # Rest는 ±0.15 rad ≈ ±8.6°
            i_limit=0.10, deadband_deg=0.5
        )
        self.pid_controller.reset()

        # IMU 사용 토글(버튼), 디바운싱 플래그
        self.use_imu = False
        self._imu_toggle_latched = False
        self.use_button = True  # 그대로 유지해도 되지만, 내부에서 래칭으로 처리

        # IMU 가드(전복/과대기울기에서 보정 OFF + PID 리셋)
        self.imu_gate = True
        self.th_off = np.deg2rad(70)  # 이 이상 기울면 보정 OFF
        self.th_on  = np.deg2rad(50)  # 이 이하로 회복되면 보정 ON

    def updateStateCommand(self, msg, state, command):
        # Update local body position based on joystick input
        # state.body_local_position[0] = msg.axes[3] * -10
        # state.body_local_position[1] = msg.axes[4] * 10
        # state.body_local_position[2] = msg.axes[7] * 10
        # 시험용

        # robot_height는 음수값이므로 -5를 곱함, min,max도 마찬가지로 음수이므로 부등호 반대
        # 나중에 robot_height를 양수로 바꾸는 작업 필요할듯?
        command.robot_height = command.robot_height + msg.axes[7] * -5
        if (command.robot_height < command.max_height):
            command.robot_height = command.max_height
        if (command.robot_height > command.min_height):
            command.robot_height = command.min_height

        # Update local body orientation based on joystick input
        state.body_local_orientation[0] = msg.axes[0] * 0.2  # Roll
        state.body_local_orientation[1] = msg.axes[1] * 0.25  #pitch

        # IMU 토글(같은 버튼 인덱스로 디바운싱)
        if msg.buttons[6] and not self._imu_toggle_latched:
            self.use_imu = not self.use_imu
            #토글 시 상태 초기화 (토글 직후 이전 상태 남아있을 수 있음)
            self.pid_controller.reset()
            self._imu_toggle_latched = True
            print(f"RESTController - Use rp compensation: {self.use_imu}")
        if not msg.buttons[6]:
            self._imu_toggle_latched = False

    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        # Copy the default stance and adjust for robot height
        temp = self.default_stance.copy() #원본 파괴 방지
        temp[2] = [command.robot_height] * 4

        # 사용자가 준 로컬 바디 기울기 적용
        roll_cmd  = state.body_local_orientation[0]
        pitch_cmd = state.body_local_orientation[1]
        rot = rotxyz(roll_cmd, pitch_cmd, 0.0)
        temp = np.matmul(rot, temp)

        # IMU 보정: 가드 + PID (언랩/LPF 포함)
        if self.use_imu:
            roll = state.imu_roll
            pitch = state.imu_pitch

            # 가드: 큰 기울기/전복 시 보정 OFF + PID 리셋
            is_inverted = getattr(state, "is_inverted", False)
            if self.imu_gate and (abs(roll) > self.th_off or abs(pitch) > self.th_off or is_inverted):
                self.imu_gate = False
                self.pid_controller.reset()
            elif (not self.imu_gate) and (abs(roll) < self.th_on and abs(pitch) < self.th_on and not is_inverted):
                self.imu_gate = True

            if self.imu_gate:
                # PID가 반환하는 보정각을 반대로 적용하여 몸을 수평에 가깝게
                corrections = self.pid_controller.run(roll, pitch)  # [roll_corr, pitch_corr]
                roll_comp  = -corrections[0]
                pitch_comp = -corrections[1]
                rot2 = rotxyz(roll_comp, pitch_comp, 0.0)
                temp = np.matmul(rot2, temp)

        return temp

    def run(self, state, command):
        # Update foot locations for the current step
        state.foot_location = self.step(state, command)
        return state.foot_location