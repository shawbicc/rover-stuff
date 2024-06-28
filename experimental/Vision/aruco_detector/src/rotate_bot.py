#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from aruco_detector.msg import direction

def callback(data):
    rotate(data.dir)

rospy.init_node('rotate_to_aruco',anonymous=True)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
sub = rospy.Subscriber('/direction',direction, callback)
#rate = rospy.Rate(100)

def rotate(target):
    twist = Twist()
    print(target)
    if target == 'Right':
        twist.angular.z = -0.5
        #print('Rotating Anti-Clockwise')

    elif target == 'Left':
        twist.angular.z = 0.5
        #print('Rotating Clockwise')

    else:
        twist.angular.z = 0
    
    pub.publish(twist)
    #rate.sleep(100)

rospy.spin()