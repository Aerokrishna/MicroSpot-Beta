import math
from trajectory_planner import trajectoryPlanner
from gait_parameters import gaitParameters
from inverse_kinematics import inverse_kinematics
# half gait period swings the diagonal legs then next half swings the other half 

# back left   Leg 1
# front left  Leg 2
# back right  Leg 3
# front right Leg 4

class walkGaitController():
    def __init__(self):
        params = gaitParameters()

        leg1_initial_pos = params.stance_period/2 # swing first leg
        leg2_initial_pos = params.gait_period - (params.stance_period/2)
        leg3_initial_pos = params.gait_period - (params.stance_period/6)
        leg4_initial_pos = params.stance_period/6

        self.leg1 = trajectoryPlanner(leg1_initial_pos) # front left and back right
        self.leg2 = trajectoryPlanner(leg2_initial_pos) # front right and back left
        self.leg3 = trajectoryPlanner(leg3_initial_pos) # front left and back right
        self.leg4 = trajectoryPlanner(leg4_initial_pos) # front right and back left
        
    def walkGait(self):
        waypoints1 = self.leg1.get_trajectory() # waypointsA is a list [x,z] for Swing first diagonal legs 
        waypoints2 = self.leg2.get_trajectory() # waypointsB is a list [x,z] for Stance first diagonal legs 
        waypoints3 = self.leg3.get_trajectory() # waypointsA is a list [x,z] for Swing first diagonal legs 
        waypoints4 = self.leg4.get_trajectory() # waypointsB is a list [x,z] for Stance first diagonal legs 

        # LEG1 
        back_left_knee = inverse_kinematics(waypoints1[0], waypoints1[1])[0]
        back_left_foot = - inverse_kinematics(waypoints1[0], waypoints1[1])[1]

        # LEG 2
        front_left_knee = inverse_kinematics(waypoints2[0], waypoints2[1])[0]
        front_left_foot = - inverse_kinematics(waypoints2[0], waypoints2[1])[1]

        # LEG 4
        front_right_knee = inverse_kinematics(waypoints3[0], waypoints3[1])[0]
        front_right_foot = - inverse_kinematics(waypoints3[0], waypoints3[1])[1]
        
        # LEG 3
        back_right_knee = inverse_kinematics(waypoints4[0], waypoints4[1])[0]
        back_right_foot = - inverse_kinematics(waypoints4[0], waypoints4[1])[1]

        return [(back_left_knee,back_left_foot),
                (front_left_knee,front_left_foot),
                (back_right_knee,back_right_foot),
                (front_right_knee,front_right_foot)] # list of tupples with angle for each leg [leg1(knee,foot)]

        

        





