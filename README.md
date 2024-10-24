# MicroSpot-Beta
This is a repository containing the code base required for cycloidal trajectory generation to simulate multiple walking gaits of a quadruped robot. This algorithm accounts for the robot's kinematic constraints to generate cycloidal motion for each leg. A straightforward position control mechanism handles switching between gaits or walking patterns based on the desired locomotion.


https://github.com/user-attachments/assets/2742313b-ec49-4aea-8731-e1572007f2a9



https://github.com/user-attachments/assets/0c39faf2-d47a-4e3c-8623-eb3678bcdc52




add an image with the leg and the cycloidal trajectory

# Functionalities and Parameters

- **Inverse Kinematics**
It takes x and z coordinates as input and computes the joint angles for each leg.

- **Trajectory Planner**
Computes a cycloidal trajectory with respect to the parameters provided. This cycloidal trajectory is discretized in the form of time_steps. The x and z coordinates of the trajectory is obtained. A complete trajectory of one leg consists of a swing and a stance phase. In the swing phase the leg traverses a cycloidal trajectory of pre-defined length and in the stance phase it is retracted through a straight line.

- **Gait Controllers**
Controls the cycle of each leg individually according to the gait required. It switches the legs between swing and stance phase and computes the joint angles synchronously.

- **Parameters file**
A parameter file to generate the desired trajectory.
![gait_params](https://github.com/user-attachments/assets/4a44259c-77aa-4eb5-b15e-3498054a5805)
1) stance_period : Time period for the leg to remain in stance phase
2) swing_period : Time period for the leg to remain in swing phase
3) gait_period : Time period of one cycle or gait
4) robot_height : height of the shoulder joint from the ground
5) step_length : Length of one step
6) step_height : Height of the step
7) time_step : time step for discretization of the trajectory






