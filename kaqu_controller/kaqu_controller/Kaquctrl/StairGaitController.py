from kaqu_controller.Kaquctrl.TrotGaitController import TrotGaitController

class StairTrotGaitController(TrotGaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        super().__init__(default_stance, stance_time, swing_time, time_step, use_imu)

        # 발 높이 조정

        # PID 보정 게인 조정 
    def step(self, state, command):
        new_foot_locations = super().step(state, command)

        # IMU 기반 회전 보정 적용
        