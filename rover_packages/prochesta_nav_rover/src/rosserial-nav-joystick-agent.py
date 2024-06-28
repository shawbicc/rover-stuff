#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class JoyToCmdVel:
    def __init__(self):
        rospy.init_node('joy_to_cmd_vel')
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber('/nav/joy', Joy, self.joy_callback)
        
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_publishers)
        
        self.last_published_twist = Twist()
        self.joy_data = None
        self.has_publishers = False
        self.waiting_for_controller = True
        
        self.button_indices = [4, 5]  # Buttons for publishing
        self.axis_linear = 1  # Axis for linear.x
        self.axis_angular = 3  # Axis for angular.z

    def check_publishers(self, event):
        publishers = rospy.get_published_topics()
        self.has_publishers = any(topic == '/nav/joy' for topic, _ in publishers)

        if not self.has_publishers:
            if self.waiting_for_controller:
                rospy.loginfo("Waiting for controller to be connected to /nav/joy")
                self.waiting_for_controller = False
            self.publish_zero_twist()
        else:
            if not self.waiting_for_controller:
                rospy.loginfo("Controller found")
                rospy.loginfo("Use the controller as follows:")
                rospy.loginfo("Press buttons 4 and 5 together to publish full axis values.")
                rospy.loginfo("Press either button 4 or 5 to publish 0.7 times the axis values.")
                self.waiting_for_controller = True

    def publish_zero_twist(self):
        zero_twist = Twist()
        if zero_twist != self.last_published_twist:
            self.cmd_vel_pub.publish(zero_twist)
            self.last_published_twist = zero_twist

    def joy_callback(self, joy_msg):
        self.joy_data = joy_msg

        if self.has_publishers:
            if self.joy_data:
                buttons = self.joy_data.buttons
                axes = self.joy_data.axes

                # Check if the specified buttons are pressed
                if buttons[self.button_indices[0]] and buttons[self.button_indices[1]]:
                    multiplier = 1.0  # Both buttons pressed
                elif buttons[self.button_indices[0]] or buttons[self.button_indices[1]]:
                    multiplier = 0.7  # One button pressed
                else:
                    multiplier = 0.0  # No buttons pressed

                twist = Twist()
                twist.linear.x = multiplier * axes[self.axis_linear]
                twist.angular.z = multiplier * axes[self.axis_angular]

                if twist != self.last_published_twist:
                    self.cmd_vel_pub.publish(twist)
                    self.last_published_twist = twist

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = JoyToCmdVel()
        node.run()
    except rospy.ROSInterruptException:
        pass
