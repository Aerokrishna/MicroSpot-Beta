import math
class gaitParameters():
    def __init__(self):
        self.stance_period = 0.8
        self.swing_period = 0.2
        self.gait_period = self.stance_period + self.swing_period
        self.robot_height = 0.18
        self.step_length = 0.08
        self.step_height = self.step_length/math.pi
        self.time_step = 0.01
        