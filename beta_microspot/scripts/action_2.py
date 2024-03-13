#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
from inverse_kinematics import inverse_kinematics


class JointStateAction(Node):
    def __init__(self):
        global x,y
        super().__init__('joint_state_action')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(String,'keyboard_input',self.input_callback,10)
        self.s_key_pressed = False
        self.joint_positions = [0.0] * 12  

        self.timer = self.create_timer(0.02, self.update_joint_state)

        # Initialize joint positions
        self.initial_x = 0.002
        self.initial_y = 0.18
        self.final_x = 0.06
        self.r = 0.03
        self.x = [0.002, 0.06, 0.06, 0.002]
        self.y = [0.18, 0.18, 0.18, 0.18]
        self.switch = [0, 0, 0, 0]
        #steps
        self.no_of_steps = 20 # number of waypoints between two points
        self.dir = [0, 1, 1, 0]
        self.steps = [0, self.no_of_steps, self.no_of_steps, 0]
        self.x_step = (self.final_x - self.initial_x)/(self.no_of_steps) #distance between each waypoints
        # self.final_y = 0.18

    def update_joint_state(self):
        # if 's' key is pressed
        if self.s_key_pressed:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = [
                'front_left_shoulder', 'front_left_leg', 'front_left_foot', #0 1 2
                'front_right_shoulder', 'front_right_leg', 'front_right_foot', #3 4 5
                'rear_left_shoulder', 'rear_left_leg', 'rear_left_foot', #6 7 8
                'rear_right_shoulder', 'rear_right_leg', 'rear_right_foot'#9 10 11
            ]

            #compute waypoints on the trajectory
            for i in range(4):    
                self.legControl(i)
            # self.legControl(1)
            self.get_logger().info(f"{self.x[0],self.y[0]}")
          
            #compute joint angles using inverse kinematics wrt waypoint coordinates
            self.joint_positions[1] = inverse_kinematics(self.x[0], self.y[0])[0]
            self.joint_positions[2] = - inverse_kinematics(self.x[0], self.y[0])[1]

            self.joint_positions[4] = inverse_kinematics(self.x[1], self.y[1])[0]
            self.joint_positions[5] = - inverse_kinematics(self.x[1], self.y[1])[1]
            
            self.joint_positions[7] = inverse_kinematics(self.x[2], self.y[2])[0]
            self.joint_positions[8] = - inverse_kinematics(self.x[2], self.y[2])[1]

            self.joint_positions[10] = inverse_kinematics(self.x[3], self.y[3])[0]
            self.joint_positions[11] = - inverse_kinematics(self.x[3], self.y[3])[1]

            self.joint_positions[0] = 0.1
            self.joint_positions[3] = -0.1

            self.joint_positions[6] = 0.1
            self.joint_positions[9] = -0.1

            #publish the joint angles
            joint_state.position = self.joint_positions[:]
            self.publisher_.publish(joint_state)
    
    def legControl(self, leg):

        # #to switch trajectory
        # if self.steps[leg] == 1 or self.steps[leg] == -self.no_of_steps:
        #     self.switch[leg] = 0
        # if self.steps[leg] == self.no_of_steps or self.steps[leg] == -1:
        #     self.switch[leg] = 1

        #to switch trajectory
        if self.steps[leg] == 0 :
            self.switch[leg] = 0
        if self.steps[leg] == self.no_of_steps:
            self.switch[leg] = 1

        #move leg forward 
        # if self.switch[leg] == 0:
        #     self.x[leg] = self.x[leg] + self.x_step
        #     if self.dir[leg] == 1: #forward circle
        #         self.y[leg] = self.initial_y - math.sqrt((self.r)**2 - (self.x[leg] - self.initial_x - self.r)**2)
        #     if self.dir[leg] == 0: #back circle
        #         self.y[leg] = self.initial_y - math.sqrt((self.r)**2 - (self.x[leg] - self.initial_x + self.r)**2)
        #     self.steps[leg] += 1
        
        if self.switch[leg] == 0:
            self.x[leg] = self.x[leg] + self.x_step
            self.y[leg] = self.initial_y - math.sqrt((self.r)**2 - (self.x[leg] - self.initial_x - self.r)**2)
            self.steps[leg] += 1
            
        #move leg backward
        else:
            self.x[leg] = self.x[leg] - self.x_step
            self.y[leg] = self.initial_y
            self.steps[leg] -= 1

    def input_callback(self, msg):
        self.get_logger().info("taking input!")
        if str(msg.data) == 's':
            self.s_key_pressed = True
            self.get_logger().info("Pressed 's' key")
        else:
            self.s_key_pressed = False
            self.get_logger().info("Released 's' key")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
