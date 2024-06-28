#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from coordinates import coordinates
from tf.transformations import euler_from_quaternion
import time

x, y = 0, 0
pi = 3.141592653589

xerr = 0
yerr = 0
initAng = 0

def newOdom(msg):
    global x, y, yaw, xerr, yerr
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
    _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    # print(yaw)

rospy.init_node("speed_controller")
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
speed = Twist()

# def initAngleCorrector():
#     initAng = yaw
#     print("before correction: ",yaw)
#     while abs(yaw) > 0.002:
#         if yaw > 0:
#             speed.angular.z = -1*initAng/2
#             pub.publish(speed)
#         elif yaw < 0:
#             speed.angular.z = -1*initAng/2
#             pub.publish(speed)

    
#     print("after correction: ",yaw)
#     speed.angular.z = 0
#     pub.publish(speed)

def allSpeedZero():
    speed.angular.z = 0
    speed.linear.x = 0
    pub.publish(speed)

def turn_left():  
    desired_yaw = yaw + pi/2  
    print(desired_yaw)
    while abs(yaw - desired_yaw) > pi/500:
        speed.angular.z = (desired_yaw - yaw) /2
        pub.publish(speed)
        # print(yaw)

    allSpeedZero()
    print("Left Turn")
    print("Final Rotation : ",yaw)
    print("Target was : ",desired_yaw)
    

def turn_right(): 
    desired_yaw = yaw - pi/2  
    while abs(yaw - desired_yaw) > pi/500:
        speed.angular.z = (desired_yaw - yaw)/2
        pub.publish(speed)
    
    allSpeedZero()
    print("Right Turn")
    print("Final Rotation : ",yaw)
    print("Target was : ",desired_yaw)


def run(goal):
    global x, y, yaw, pub, speed , xerr, yerr
    allSpeedZero()
    print("Goal is: ",goal)
    print("Current Position: ",x,y)
    print("Again started")
    if goal.x  < x:
        turn_left()
        turn_left()
        print("Line 64")
    
    while abs(goal.x - x) > 2:
        speed.linear.x = 0.2
        pub.publish(speed)
        # print("Line 69 modified")

    while abs(goal.x - x) > 0.05:
        speed.linear.x = (goal.x - x)/5
        pub.publish(speed)
        # print("Line 69")   
    print(x,y)
    allSpeedZero()
    
    if goal.y < y:
        turn_right()

    elif goal.y > y:
        turn_left()
    print("After Rotation: ",x,y)
    allSpeedZero()

    while abs(goal.y - y) > 2:
        speed.linear.x = 0.5
        pub.publish(speed)
        # print("Line 108")

    while abs(goal.y - y) > 0.05:
        speed.linear.x = abs(goal.y - y)/5
        pub.publish(speed)
        # print("Line 113")
         
    print(x,y)
    allSpeedZero()

    if goal.y < 0:
        turn_left()
        print("Entered Left")

    elif goal.y > 0:
        turn_right()
        print("Entered Right")
    
    # fix_x()
        print("After Rotation: ",x,y)
    
    # xerr = abs(goal.x - x)
    # yerr = abs(goal.y - y)
    #print(xerr,yerr)
    print("1st Goal Done! Coordinates: ", x,y)

   
for coordinate in coordinates:
    
    goal = Point(x=coordinate[0], y=coordinate[1])
    run(goal)

