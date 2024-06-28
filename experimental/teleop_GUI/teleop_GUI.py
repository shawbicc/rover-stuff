

from kivy.app import App
from kivy.lang import Builder
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.slider import Slider
from kivy.uix.button import Button
from kivy.properties import ObjectProperty


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import datetime
import math


MAX_LIN_VEL = 1
MAX_ANG_VEL = 1



class teleop_GUIApp(App):
    def __init__(self, nav_pub, arm_pub):
        super().__init__()
        ########## ROS STUFF ################
        self.nav_pub = nav_pub
        self.arm_pub = arm_pub
        self.rate = rospy.Rate(1)

        ############ WHEEL ATTRIBUTES ################
        self.speed_slider_value = 50.0
        
        self.target_lin_vel = 0
        self.target_ang_vel = 0

        ############ ARM ATTRIBUTES ##################
        self.arm_joint_angles = [0, 0, 0, 0, 0, 0]
        
    def on_start(self):
        self.speed_values_field = self.root.ids.speed_values_field



    def map_slider_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    ############ CMD_VEL PUBLISHER ############################
    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.target_lin_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.target_ang_vel
        self.nav_pub.publish(twist)

    ################ FUNCTION FOR JOINT ANGLE SLIDER CHANGE ##################
    def arm_slider_changed(self, id, value):
        self.arm_joint_angles[id-1] = value
        print(f"Joint-{id}: {self.arm_joint_angles[id-1]:.0f}")


    def build(self):
        return Builder.load_file("teleop_GUI.kv")
    
    ######################################################### WHEEL FUNCTIONS ############################################################

    ########## FUNCTION ON SLIDER ADJUSTMENT ##############
    def speed_value_adjusted(self, value):
        self.speed_slider_value = value
        
        if self.target_lin_vel > 0:
            self.target_lin_vel = self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_LIN_VEL)
        if self.target_lin_vel < 0:
            self.target_lin_vel = -(self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_LIN_VEL))
        
        if self.target_ang_vel > 0:
            self.target_ang_vel = self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_ANG_VEL)
        if self.target_ang_vel < 0:
            self.target_ang_vel = self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_ANG_VEL)
        
        self.publish_twist()

        self.speed_values_field.text = f"Linear: {self.target_lin_vel:.2f} m/s\nAngular: {self.target_ang_vel:.2f} rad/s"
        print(f"Speed: {value:.2f}%")


    ############ FUNCTION FOR FORWARD BUTTON ################
    def forward(self):
        self.target_lin_vel = self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_LIN_VEL)
        self.target_ang_vel = 0
        self.publish_twist()
        self.speed_values_field.text = f"Linear: {self.target_lin_vel:.2f} m/s\nAngular: {self.target_ang_vel:.2f} rad/s"
        print(f"Forward")

    ############ FUNCTION FOR BACKWARD BUTTON ################
    def backward(self):
        print(f"Backward")
        self.target_lin_vel = -(self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_LIN_VEL))
        self.target_ang_vel = 0
        self.publish_twist()
        self.speed_values_field.text = f"Linear: {self.target_lin_vel:.2f} m/s\nAngular: {self.target_ang_vel:.2f} rad/s"


    ############ FUNCTION FOR Turn Left BUTTON ################
    def left(self):
        print(f"Turn Left")
        self.target_lin_vel = 0
        self.target_ang_vel = -(self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_ANG_VEL))
        self.publish_twist()
        self.speed_values_field.text = f"Linear: {self.target_lin_vel:.2f} m/s\nAngular: {self.target_ang_vel:.2f} rad/s"


    ############ FUNCTION FOR Turn Right BUTTON ################
    def right(self):
        print(f"Turn Right")
        self.target_lin_vel = 0
        self.target_ang_vel = self.map_slider_value(self.speed_slider_value, 0, 100, 0, MAX_ANG_VEL)
        self.publish_twist()
        self.speed_values_field.text = f"Linear: {self.target_lin_vel:.2f} m/s\nAngular: {self.target_ang_vel:.2f} rad/s"

    ############ FUNCTION FOR STOP BUTTON ################
    def stop(self):
        print(f"Stop")
        self.target_lin_vel = 0
        self.target_ang_vel = 0
        self.publish_twist()
        self.speed_values_field.text = f"Linear: {self.target_lin_vel} m/s\nAngular: {self.target_ang_vel} rad/s"


    ################################################################ ARM FUNCTIONS ##################################################################
        
    def joint_angle_publish(self):
        pass
        

    ######### FUNCTION TO OPEN GRIPPER ####################
    def open_gripper(self):
        pass

    ######### FUNCTION TO CLOSE GRIPPER ####################
    def close_gripper(self):
        pass

    ######### FUNCTION TO TAKE ARM TO REST POSITION ####################
    def rest_position(self):
        pass



if __name__ == "__main__":
    ############### FUNCTIONALITY FOR CONTROLLER NODE #################
    rospy.init_node("teleop_gui")
    nav_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    arm_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    
    app = teleop_GUIApp(nav_pub=nav_pub, arm_pub=arm_pub)
    app.run()
