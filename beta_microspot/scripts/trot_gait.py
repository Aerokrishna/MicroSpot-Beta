import math
from trajectory_planner import trajectoryPlanner
from gait_parameters import gaitParameters
from inverse_kinematics import inverse_kinematics
# half gait period swings the diagonal legs then next half swings the other half 

# back left   Leg 1
# front left  Leg 2
# back right  Leg 3
# front right Leg 4

class trotGaitController():
    def __init__(self):
        params = gaitParameters()
        legSetA_initial_pos = params.stance_period/2 # starts of with swing
        legSetB_initial_pos = params.gait_period - (params.stance_period/2) # starts of with stance

        self.legSetA = trajectoryPlanner(legSetA_initial_pos) # front left and back right
        self.legSetB = trajectoryPlanner(legSetB_initial_pos) # front right and back left
        
    def trotGait(self):
        waypointsA = self.legSetA.get_trajectory() # waypointsA is a list [x,z] for Swing first diagonal legs 
        waypointsB = self.legSetB.get_trajectory() # waypointsB is a list [x,z] for Stance first diagonal legs 

        # STANCE-FIRST LEGS

        # LEG1 
        back_left_knee = inverse_kinematics(waypointsB[0], waypointsB[1])[0]
        back_left_foot = - inverse_kinematics(waypointsB[0], waypointsB[1])[1]
        # LEG 4
        front_right_knee = inverse_kinematics(waypointsB[0], waypointsB[1])[0]
        front_right_foot = - inverse_kinematics(waypointsB[0], waypointsB[1])[1]

        # SWING-FIRST LEGS

        # LEG 2
        front_left_knee = inverse_kinematics(waypointsA[0], waypointsA[1])[0]
        front_left_foot = - inverse_kinematics(waypointsA[0], waypointsA[1])[1]
        # LEG 3
        back_right_knee = inverse_kinematics(waypointsA[0], waypointsA[1])[0]
        back_right_foot = - inverse_kinematics(waypointsA[0], waypointsA[1])[1]

        return [(back_left_knee,back_left_foot),
                (front_left_knee,front_left_foot),
                (back_right_knee,back_right_foot),
                (front_right_knee,front_right_foot),waypointsB,waypointsA] # list of tupples with angle for each leg [leg1(knee,foot)]

        

        





