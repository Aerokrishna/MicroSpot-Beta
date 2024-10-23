# MicroSpot-Beta
This is a repository containing the code base required for cycloidal trajectory generation to simulate multiple walking gaits of a quadruped robot. 

add videos demonstrating atleast 2 walking gaits

This algorithm accounts for the robot's kinematic constraints to generate cycloidal motion for each leg. A straightforward position control mechanism handles switching between gaits or walking patterns based on the desired locomotion.

add an image with the leg and the cycloidal trajectory

# Functionalities and Parameters

- **Parameters file**
One stop station to edit the required parameters regarding the robots motion. This includes the gait period, swing period step height step length etc.

- **Inverse Kinematics**
Takes x and z coordinates as input and gives out the knee and foot angles as the output. (Currently it computes in 2 dimensions only)

- **Trajectory Planner**
Computes a cycloidal trajectory with respect to the parameters provided. This cycloidal trajectory is formed in form of time_steps and x and z coordinates obtained. It alternates between a cycloid and straight line for swing and stance phases of a gait cycle.

- **Gait Controllers**
(walking_gait.py, double_walking_gait.py, trot_gait.py)
Controls all the 4 legs individully in a pre-defined pattern switching between swing and stance phase. Inverse kinematics for the coordinates computed by trajectory_planner is used to output 8 joint angles.

action_3.py
Main working code where the output angles of the gait controllers are stored in a variable and published to joint_states topic.


