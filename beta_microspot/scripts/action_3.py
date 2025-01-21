#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
from trot_gait import trotGaitController
from double_walking_gait import DwalkGaitController
from walking_gait import walkGaitController
from visualization_msgs.msg import Marker
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
        self.marker_publisher_ = self.create_publisher(Marker,'/visualization_marker',10)
        self.marker_publisher_2 = self.create_publisher(Marker,'/visualization_marker2',10)

        self.marker_timer_ = self.create_timer(0.1, self.marker_timer)


        self.s_key_pressed = False
        self.joint_positions = [0.0] * 12  
        self.timer = self.create_timer(gaitParameters().time_step, self.update_joint_state)
        self.trotGait = trotGaitController()
        self.DwalkGait = DwalkGaitController()
        self.walkGait = walkGaitController()

        self.viz_leg1 = []
        self.viz_leg2 = []

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

            self.joint_angles = self.trotGait.trotGait()
            # self.joint_angles = self.DwalkGait.DwalkGait()
            # self.joint_angles = self.walkGait.walkGait()
            self.viz_leg1.append(self.joint_angles[4])
            # print(self.joint_angles[4])
            self.viz_leg2.append(self.joint_angles[5])

            
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

            self.joint_positions[0] = 0.0
            self.joint_positions[3] = 0.0
            self.joint_positions[6] = 0.0
            self.joint_positions[9] = 0.0

            #publish the joint angles
            joint_state.position = self.joint_positions[:]
            self.publisher_.publish(joint_state)

    def marker_timer(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = "base_footprint" # wrt which frame we are taking the coordinates
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.type = Marker.SPHERE
    
        marker_msg.scale.x = 0.01
        marker_msg.scale.y = 0.01
        marker_msg.scale.z = 0.01

        marker_msg.color.r = 155.0
        marker_msg.color.g = 233.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0

        for i in range(0,len(self.viz_leg1)):
          
            marker_msg.pose.position.x = self.viz_leg1[i][0] + 0.093
            marker_msg.pose.position.y = -0.1
            marker_msg.pose.position.z = -self.viz_leg1[i][1] + 0.22
            marker_msg.pose.orientation.w = 0.0

            marker_msg.id = i
            self.marker_publisher_.publish(marker_msg)
        
        marker_msg2 = Marker()
        marker_msg2.header.frame_id = "base_footprint" # wrt which frame we are taking the coordinates
        marker_msg2.header.stamp = self.get_clock().now().to_msg()
        marker_msg2.type = Marker.SPHERE
    
        marker_msg2.scale.x = 0.01
        marker_msg2.scale.y = 0.01
        marker_msg2.scale.z = 0.01

        marker_msg2.color.r = 155.0
        marker_msg2.color.g = 233.0
        marker_msg2.color.b = 0.0
        marker_msg2.color.a = 1.0

        marker_msg2.color.r = 0.0
        marker_msg2.color.g = 233.0
        marker_msg2.color.b = 155.0
        marker_msg2.color.a = 1.0

        for i in range(0,len(self.viz_leg2)):
          
            marker_msg2.pose.position.x = self.viz_leg2[i][0] - 0.093
            marker_msg2.pose.position.y = -0.1
            marker_msg2.pose.position.z = -self.viz_leg2[i][1] + 0.22
            marker_msg2.pose.orientation.w = 0.0

            marker_msg2.id = i 
            self.marker_publisher_2.publish(marker_msg2)

        # marker_msg.pose.position.x = 1.0
        # marker_msg.pose.position.y = 0.0
        # marker_msg.pose.position.z = 1.0
        # marker_msg.id = 1
        # self.marker_publisher_.publish(marker_msg)

        
    def input_callback(self, msg : String):
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
