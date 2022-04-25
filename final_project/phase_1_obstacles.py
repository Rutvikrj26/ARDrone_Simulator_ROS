from turtle import circle
from cv_bridge import CvBridge
import rosbag
import subprocess, yaml
import cv2 as cv
import numpy as np
import time

# Camera Intrinsic matrix
K = np.array([[604.62, 0.0, 320.5], [0.0, 604.62, 180.5], [0.0, 0.0, 1.0]])

# Perform circle detection.
def circleDetection(msg):

    # Initialization
    detected = False
    bridge = CvBridge()
    detected_circles = np.uint16(np.zeros(3))

    # Converting ROS image to opencv image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Blurring the image for improved circle detection accuracy
    cv_image = cv.medianBlur(cv_image, 7)

    # Converting image to hsv type
    hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    # Creating mask for blue color detection (red obstacles - opencv switches blue and red colors - BGR scheme)
    lower = np.array([115, 180, 50], dtype="uint8")
    upper = np.array([126, 255, 255], dtype="uint8")
    mask = cv.inRange(hsv_image, lower, upper)  

    # Creating gray image for hough transform
    det_img = cv.bitwise_and(cv_image, cv_image, mask = mask)
    det_img = cv.cvtColor(det_img, cv.COLOR_BGR2GRAY)
    canny_img = cv.Canny(det_img, 50, 240)

    # Hough circle transform for circle detection
    circles = cv.HoughCircles(canny_img, cv.HOUGH_GRADIENT, dp = 1, minDist = 10, param1 = 50, param2 = 26, minRadius = 20, maxRadius = 110)

    # Marking the detected circles
    if circles is None:
        pass
    else:
        detected_circles = np.uint16(np.around(circles))
        circle_no = 0
        for (x, y, r) in detected_circles[0, :]:
            cv.circle(cv_image, (x, y), r, (0, 0, 0), 3)
            cv.circle(cv_image, (x, y), 1, (0, 0, 0), 3)
            circle_no += 1
        detected = True

    return cv_image, detected_circles, detected

# Perform coordinate transformation
def Coordinatetransform(x_s, y_s, prev_msg):

    #Inverse pixel transform.
    xNyN = np.dot(np.linalg.inv(K),[[[x_s], [y_s], [1]]])

    #Inverse normalized plane projection.
    x=xNyN[0]*prev_msg.transform.translation.z
    y=xNyN[1]*prev_msg.transform.translation.z

    #Target location in body coordinate frame
    target = np.dot(np.linalg.inv(Tcb),[x,y,previous_msg.transform.translation.z,1])

    # Target location in global coordinate frame
    result = np.add(np.array([target[0],target[1],target[2]]),np.array([previous_msg.transform.translation.x,previous_msg.transform.translation.y,
                    previous_msg.transform.translation.z]))

    return result

# Read bag file for messages.
bag = rosbag.Bag('FinalProject.bag')

# Read summary of bag file to test.
# This is a dictionary of all the data.
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', 'FinalProject.bag'], stdout=subprocess.PIPE).communicate()[0])

# Create variable based on number of images
num_images = info_dict["topics"][0]["messages"]
num = num_images

# Variable for appending topic data
data = []

#Tcb matrix
Tcb = np.array([[0.0, -1.0, 0.0, 0.0],[-1.0, 0.0, 0.0, 0.0125],[0.0, 0.0, -1.0, -0.025],[0.0, 0.0, 0.0, 1.0]])

for topic, msg, t in bag.read_messages(topics=['/vicon/ARDroneCarre/ARDroneCarre','/ardrone/bottom/image_raw']):

    if (topic=='/ardrone/bottom/image_raw'):

        img = circleDetection(msg)
        circle_list = circleDetection(msg)[1]
        detection = circleDetection(msg)[2]

        #Perform localization.
        if detection:
            for i in range(len(circle_list[0])):

                # Center of the detected circle
                result_c = Coordinatetransform(circle_list[0][i][0], circle_list[0][i][1], previous_msg)

                # Radius of the detected circle
                result_r_0 = Coordinatetransform(0, 0, previous_msg)
                point_0 = np.array((result_r_0[0] ,result_r_0[1], result_r_0[2]))

                result_r_1 = Coordinatetransform(0, circle_list[0][i][2], previous_msg)
                point_1 = np.array((result_r_1[0] ,result_r_1[1], result_r_1[2]))

                dist = np.linalg.norm(point_0 - point_1)
                print([result_c, dist])

                # Video text
                if i == 0:
                    cv.putText(img[0], "("+str(result_c[0]) + "," + str(result_c[1]) + "," + str(result_c[2])+")", (50, 50),
                            cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    cv.putText(img[0], "("+str(result_c[0]) + "," + str(result_c[1]) + "," + str(result_c[2])+")", (50, 70),
                            cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)

        data.append({previous_topic:previous_msg})
        data.append({topic:img[0]})

    previous_msg = msg
    previous_topic = topic

fourcc = cv.VideoWriter_fourcc('m','p','4','v')
video = cv.VideoWriter('video.avi', fourcc, 30, (640, 360),True)

for i in range(num_images):
    if (i%2!=0):
        video.write(data[i].get('/ardrone/bottom/image_raw'))
video.release()