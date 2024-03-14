#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
from trot_gait import trotGaitController
from double_walking_gait import DwalkGaitController
from walking_gait import walkGaitController
from gait_parameters import gaitParameters

# back left   Leg 1
# front left  Leg 2
# back right  Leg 3
# front right Leg 4

class JointStateAction(Node):
    def __init__(self):
        super().__init__('joint_state_action')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(String,'keyboard_input',self.input_callback,10)
        self.s_key_pressed = False
        self.joint_positions = [0.0] * 12  
        self.timer = self.create_timer(gaitParameters().time_step, self.update_joint_state)
        self.trotGait = trotGaitController()
        self.DwalkGait = DwalkGaitController()
        self.walkGait = walkGaitController()

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

            # self.joint_angles = self.trotGait.trotGait()
            # self.joint_angles = self.DwalkGait.DwalkGait()
            self.joint_angles = self.walkGait.walkGait()
            
            # LEG 1 BL
            self.joint_positions[7] = self.joint_angles[0][0]
            self.joint_positions[8] = self.joint_angles[0][1]

            # LEG 2 FL
            self.joint_positions[1] = self.joint_angles[1][0]
            self.joint_positions[2] = self.joint_angles[1][1]

            # LEG 3 BR
            self.joint_positions[10] = self.joint_angles[2][0]
            self.joint_positions[11] = self.joint_angles[2][1]

            # LEG 4 FR
            self.joint_positions[4] = self.joint_angles[3][0]
            self.joint_positions[5] = self.joint_angles[3][1]

            #publish the joint angles
            joint_state.position = self.joint_positions[:]
            self.publisher_.publish(joint_state)

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
