#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
from inverse_kinematics import inverse_kinematics

class JointStateAction(Node):
    def __init__(self):
        super().__init__('joint_state_action')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(String,'keyboard_input',self.input_callback,10)
        self.s_key_pressed = False
        self.joint_positions = [0.0] * 12  

        # Initialize joint angles
        self.joint_positions[4] = 0.74
        self.joint_positions[5] = - 1.35
        self.joint_positions[7] = 0.74
        self.joint_positions[8] = - 1.35

        self.joint_positions[1] = 0.036
        self.joint_positions[2] = - 0.96
        self.joint_positions[10] = 0.036
        self.joint_positions[11] = - 0.96

        self.timer = self.create_timer(0.007, self.update_joint_state)

        # Initialize joint positionss
        self.initial_x1 = 0.002
        self.initial_y1 = 0.18

        self.initial_x2 = 0.10
        self.initial_y2 = 0.18

        self.x1 = self.initial_x1
        self.y1 = self.initial_y1

        self.x2 = self.initial_x2
        self.y2 = self.initial_y2

        self.cnt1 = 0
        self.cnt2 = 0
 
    def update_joint_state(self):

        # if 's' key is pressed
        if self.s_key_pressed:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = [
                'front_left_shoulder', 'front_left_leg', 'front_left_foot',
                'front_right_shoulder', 'front_right_leg', 'front_right_foot',
                'rear_left_shoulder', 'rear_left_leg', 'rear_left_foot',
                'rear_right_shoulder', 'rear_right_leg', 'rear_right_foot'
            ]

            # Computing trajectory waypoints for diagonal 1
            if 0.098 <= self.x1 <= 0.10:
               self.cnt1+=1
               if self.cnt1>1:
                   self.cnt1 = 1

            if self.cnt1 == 1 and 0.006<=self.x1<0.009:
                self.cnt1 = 0

            if self.cnt1 == 0:
                self.x1 = self.x1 + 0.001
                self.y1 = self.initial_y1 - math.sqrt(0.0025 - (self.x1 - self.initial_x1 - 0.05)**2)

            if self.cnt1 == 1:
                 self.x1 = self.x1 - 0.001
                 self.y1 = self.y1

            # Computing trajectory waypoints for diagonal 2
            if 0.098 <= self.x2 <= 0.10:
               self.cnt2 = 0
               if self.cnt2>1:
                   self.cnt2 = 1

            if self.cnt2 == 0 and 0.006<=self.x2<0.009:
                self.cnt2 += 1

            if self.cnt2 == 0:
                self.x2 = self.x2 - 0.001
                self.y2 = self.y2

            if self.cnt2 == 1:
                self.x2 = self.x2 + 0.001
                self.y2 = self.initial_y2 - math.sqrt(0.0025 - (self.x2 - self.initial_x2 + 0.05)**2)

            # Publish the respective joint angles wrt the waypoints using Inverse Kinematics
            self.joint_positions[4] = inverse_kinematics(self.x1, self.y1)[0]
            self.joint_positions[5] = - inverse_kinematics(self.x1, self.y1)[1]
            self.joint_positions[7] = inverse_kinematics(self.x1, self.y1)[0]
            self.joint_positions[8] = - inverse_kinematics(self.x1, self.y1)[1]

            self.joint_positions[1] = inverse_kinematics(self.x2,self.y2)[0]
            self.joint_positions[2] = - inverse_kinematics(self.x2,self.y2)[1]
            self.joint_positions[10] = inverse_kinematics(self.x2,self.y2)[0]
            self.joint_positions[11] = - inverse_kinematics(self.x2,self.y2)[1]

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
