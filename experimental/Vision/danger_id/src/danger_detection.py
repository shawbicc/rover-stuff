#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np 
from danger_id.msg import detection
from geometry_msgs.msg import Twist

msg = detection()
vel_msg = Twist()

paramPath = '..//Camera Calibration//calibration.npz' 
data = np.load(paramPath)
camera_matrix = data['camMatrix']
dist_coeffs = data['distCoeff']

# ArUco marker size in cm
marker_size = 10.2  # Adjust this based on the actual size of your ArUco marker

danger_ids = [0, 8]

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()


def detect_danger():
    cap = cv2.VideoCapture(0)  

    while True:
        ret, frame = cap.read()

    # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
        corners, ids, rejected = aruco.ArucoDetector(aruco_dict, parameters).detectMarkers(gray)

        danger_ids_found = []
        danger_corners_found = []

        if ids is not None:
            for i in range(len(ids)):
                if ids[i] in danger_ids:
                    danger_ids_found.append(ids[i])
                    danger_corners_found.append(corners[i])
        else:
            msg.detected = False
            msg.id = -1        
            msg.distance = 0
            print(msg)
            pub_danger.publish(msg)

        if len(danger_ids_found) > 0:
            for i in range(len(danger_ids_found)):
                # Estimate the pose of the marker
                    # Extract marker corners
                    image_points = danger_corners_found[i].reshape(-1, 2)

                # Define object points for the marker
                    object_points = np.array([[-marker_size/2, -marker_size/2, 0],
                                [marker_size/2, -marker_size/2, 0],
                                [marker_size/2, marker_size/2, 0],
                                [-marker_size/2, marker_size/2, 0]], dtype=np.float32)

                    _, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)


                # Draw the detected ArUco marker and its axis on the image
                    aruco.drawDetectedMarkers(frame, corners)
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 1)

                # Calculate the distance from the camera to the ArUco marker
                    distance = np.linalg.norm(tvec)/1.66

                    msg.detected = True
                    msg.id = int(danger_ids_found[i][0])
                    msg.distance = float(distance)

                    if distance < 70:
                        vel_msg.linear.x = -10
                        pub_cmd_vel.publish(vel_msg)

                    print(msg)
                    pub_danger.publish(msg)           
        else:
            msg.detected = False        
            msg.id = -1        
            msg.distance = 0
            print(msg)
            pub_danger.publish(msg)


        cv2.imshow("ArUco Marker Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # rate.sleep()

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("danger_detection_node", anonymous=True)
    
    pub_danger = rospy.Publisher('/danger_detection_topic', detection, queue_size = 10)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    rate = rospy.Rate(10)
    detect_danger()
    