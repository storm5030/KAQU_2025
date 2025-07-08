import numpy as np

# 현재 어떤 phase에 있고 얼마나 지났는지, 어떤 다리가 접지인지 스윙인지
class GaitController(object):
    def __init__(self, stance_time, swing_time, time_step, contact_phases, default_stance):
        self.stance_time = stance_time
        self.swing_time = swing_time
        self.time_step = time_step
        self.contact_phases = contact_phases
        self.def_stance = default_stance

    @property
    def default_stance(self):
        return self.def_stance

    @property  # stance 행동이 가지는 틱(시간)
    def stance_ticks(self):
        return int(self.stance_time / self.time_step)

    @property  # swing 행동이 가지는 틱(시간)
    def swing_ticks(self):
        return int(self.swing_time / self.time_step)

    @property  # contact_phase에 맞는 시간을 나열
    def phase_ticks(self):
        temp = []
        for i in range(len(self.contact_phases[0])):
            if 0 in self.contact_phases[:, i]:
                temp.append(self.swing_ticks)
            else:
                temp.append(self.stance_ticks)
        return temp

    @property  # 나열한 거 합치기 (주기 틱을 표현)
    def phase_length(self):
        return sum(self.phase_ticks)

    # 현재 몇번째 phase에 있는지
    def phase_index(self, ticks):
        """ Calculate, which part of the gait cycle the robot should be in """
        phase_time = ticks % self.phase_length  # 틱에서 주기나누고 나머지
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):  # 행동의 열(phase의 개수)
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False

    # 현재 phase에서 몇틱 지났는지
    def subphase_ticks(self, ticks):
        """ Calculate the number of ticks (timesteps)
        since the begining of the current phase """
        phase_time = ticks % self.phase_length
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + phase_ticks[i]
                return subphase_ticks
        assert False

    # 어떤 다리가 접지이고 스윙인지 나열 ex)1010
    def contacts(self, ticks):
        """ Calculate which feet should be in contact """
        return self.contact_phases[:, self.phase_index(ticks)]