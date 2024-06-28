#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import numpy as np
import serial


ser = serial.Serial('/dev/ttyUSB0', 9600) # SET COM PORT ACCORDING TO ARDUINO

class JoyToTwist:
    def __init__(self):
        # Initialize the node
        rospy.init_node('joy_to_serial_node')

        # Create a subscriber for the /joy topic
        rospy.Subscriber('/joy', Joy, self.joystick_callback)
		# Controller variables
        self.SCALE = 1
        self.SCALING_FACTOR = 0
        self.prev_joy_data = None

    def joystick_callback(self, data):
        # publish if the joystick data is different than the previous one
        
        # current_joy_data = np.array(data.axes[0] + data.buttons[4] + data.buttons[5])
        current_joy_data = np.array([round(data.axes[0]), data.buttons[7], data.buttons[6], data.buttons[8], data.buttons[9]])
        if self.prev_joy_data is None or not np.array_equal(self.prev_joy_data, current_joy_data):

            # increase SCALE if UP BUTTON is pressed
            if data.buttons[9] > 0.9 and self.SCALE < 1.0:
                self.SCALE = self.SCALE + self.SCALING_FACTOR

            # reduce SCALE if DOWN BUTTON is pressed
            if data.buttons[8] > 0.9 and self.SCALE > 0.0:
                self.SCALE = self.SCALE - self.SCALING_FACTOR

            if self.SCALE > 1.0:
                self.SCALE = 1.0
            if self.SCALE < -1.0:
                self.SCALE = -1.0

            # These indices might need adjustment depending on your joystick

            x = (data.buttons[7] - data.buttons[6]) * self.SCALE  # Scale as necessary
            if x > 1.0:
                x = 1.0
            if x < -1.0:
                x = -1.0
            
            z = data.axes[0] * (-self.SCALE)  # Scale as necessary
            if z > 1.0:
                z = 1.0
            if z < -1.0:
                z = -1.0

            # Control for base, shoulder, elbow
            # b = (data.buttons[4] * -1) + data.buttons[5]
            # s = int(data.axes[2])
            # e = (data.buttons[2] * -1) + data.buttons[0]


        # Publish the Twist message to the /cmd_vel topic
            ser.write("x {}".format(x))
            ser.write("z {}".format(z))
            # ser.write("b {}".format(b))
            # ser.write("s {}".format(s))
            # ser.write("e {}".format(e))
            rospy.loginfo("Published to {}: x:{}, z:{}".format('/dev/ttyUSB0', x, z))

            # update the previous data
        self.prev_joy_data = current_joy_data

if __name__ == '__main__':
    try:
        joy_to_twist = JoyToTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

