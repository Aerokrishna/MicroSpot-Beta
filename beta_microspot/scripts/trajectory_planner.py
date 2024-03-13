import math

class trajectory_planner():
    def __init__(self):

        self.step_length = 0.08 # S
        self.step_height = self.step_length/math.pi  # H
        self.robot_height = 0.18 # l

        self.swing_period = 2        # Ts
        self.stance_period = 2       # Ty

        self.gait_period = self.swing_period + self.stance_period # T

        #self.gait_period = 0
        #slef.swing_stance_ratio = 1
        #self.swing_period = self.swing_stance_ratio * self.stance_period

        self.time_counter = 0 # t
        self.time_step = 0.01
        self.alpha = 0
        self.tau = 0
    
    def get_trajectory(self):
        self.alpha = self.time_counter - (self.stance_period/2)
        self.tau = self.alpha/self.swing_period

        # Z
        if self.time_counter <= self.stance_period/2:
            z = self.robot_height
        if self.stance_period/2 < self.time_counter < (self.stance_period/2) + self.swing_period:
            z = - ((self.step_height/2) * (1 - math.cos(2 * math.pi * self.tau))) + self.robot_height
        if self.time_counter >= self.swing_period + (self.stance_period/2):
            z = self.robot_height

        # X
        if self.time_counter <= self.stance_period/2:

            x = -self.step_length * (self.time_counter/self.stance_period)

        if self.stance_period/2 < self.time_counter < (self.stance_period/2) + self.swing_period:
            x = ((self.step_height/2) * ((2 * math.pi * self.tau) - math.sin(2 * math.pi * self.tau))) - (self.step_length/2)

        if self.time_counter >= self.swing_period + (self.stance_period/2):
            x = self.step_length * ((self.gait_period - self.time_counter)/self.stance_period)

        self.time_counter += self.time_step
        if self.time_counter >= self.gait_period:
            self.time_counter = 0
        if x == 0:
            x = 0.001
        return [x,z]
