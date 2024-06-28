#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv2 import aruco
from aruco_detector.msg import direction

################ROS Initialization Codes###################

rospy.init_node("directional_feedback",anonymous=True)
rate = rospy.Rate(10)
pub = rospy.Publisher("/direction",direction, queue_size=10)

###########################################################

F = direction()      #Custom Data Type

accepted_ids = np.array([10, 15, 9, 25])      #Ekhane Prioritywise Likhte Hobe

cap = cv2.VideoCapture(0)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
left_grid = frame_width/3
right_grid = 2*left_grid         #If face RIGHT-LEFT problem, swap left_grid with right_grid

while True:
    ret, fr = cap.read()
    frame = fr #cv2.flip(fr, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    # param_markers = aruco.DetectorParameters()
    # detector = aruco.ArucoDetector(marker_dict, param_markers)
    # corners, ids, reject = detector.detectMarkers(gray)

    #Uporer code ta onno version er jonno. Nicherta problem korle uporerta check korte hobe.
    dictionary = cv2.aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    
    if ids is not None:
        count = 0
        for accepted_id in accepted_ids:
            for id, corner in zip(ids, corners): 
                center_x = None
                center_y = None
                if id == accepted_id :
                    left_top, right_top, right_bottom, left_bottom  = corner[0]
                    center = (left_top + right_top + right_bottom + left_bottom)/4
                    center_x = center[0]
                    center_y = center[1]
                    F.x = center_x
                    F.y = center_y
                    count = count +1
                    
                if center_x == None and center_y == None:
                    continue

                if center_x <= left_grid:
                    F.dir = "Left"
                    
                elif center_x >= right_grid:
                    F.dir = "Right"
                    
                else: #center_x > left_grid and center_x < right_grid:
                    F.dir = "Middle"

                ID = str(id)
                #print(f"Center of marker {id}: ({center_x}, {center_y})")
                cv2.rectangle(frame, (int(left_top[0]), int(left_top[1])), (int(right_bottom[0]), int(right_bottom[1])), (0, 0, 255), 2)
                cv2.putText(frame, ID, (int(center_x), int(center_y*1.2)), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, (0,0,255), 2, cv2.LINE_AA)
                pub.publish(F)
            
            if count != 0:
                break
          
    cv2.line(frame, (int(left_grid), 2), (int(left_grid), int(frame_height)), (0, 255, 0), 2)
    cv2.line(frame, (int(right_grid), 2), (int(right_grid), int(frame_height)), (0, 255, 0), 2)
    #frame = cv2.flip(frame,1)

    cv2.imshow("Aruco Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

while (not rospy.is_shutdown()):
    #rospy.loginfo("Printing")
    rate.sleep()

