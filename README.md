# MicroSpot-Beta
Quadruped project for IEEE NITK.

Contains 2 seperate files beta_microspot and beta_microspot_sim.
beta_microspot_sim contains files necessary to simulate and visualize the robot on Rviz.
beta_microspot contains the python scripts in ROS to control the robot.

SCRIPTS
gait_parameters.py
One stop station to edit the required parameters regarding the robots motion. This includes the gait period, swing period step height step length etc.

inverse_kinematics.py
Takes x and z coordinates as input and gives out the knee and foot angles as the output. (Currently it computes in 2 dimensions only)

trajectory_planner.py
Computes a cycloidal trajectory with respect to the parameters provided. This cycloidal trajectory is formed in form of time_steps and x and z coordinates obtained. It alternates between a cycloid and straight line for swing and stance phases of a gait cycle.

Gait Controllers (walking_gait.py, double_walking_gait.py, trot_gait.py)
Controls all the 4 legs individully in a pre-defined pattern switching between swing and stance phase. Inverse kinematics for the coordinates computed by trajectory_planner is used to output 8 joint angles.

action_3.py
Main working code where the output angles of the gait controllers are stored in a variable and published to joint_states topic.

input_detector.py
Takes input from the keyboard and returns a string.


