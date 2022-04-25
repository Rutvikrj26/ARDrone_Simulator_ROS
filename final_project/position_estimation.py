# Python Script File for Position Estimation of the Green Circles in Lab 3

## Importing python Libraries
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

## Importing Ros Related Libraries
import rospy
import rosbag
from std_msgs.msg import String, Float32, Int32

bag_data = rosbag.Bag('FinalProject.bag')
bridge = CvBridge()
K = np.matrix([[ 604.62,   0.00000000e+00,   320.5],[  0.00000000e+00,  604.62,  180.5],[  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
d = np.array([])
T_cb = np.matrix([[0, -1, 0, 0.00], [-1, 0, 0, 0.00], [0.0, 0.0, -1, 0.00], [0.00, 0.00, 0.00, 1.00]])

C1 = np.array([[0,0]])
C2 = np.array([[0,0]])
C3 = np.array([[0,0]])
C4 = np.array([[0,0]])
C5 = np.array([[0,0]])
C6 = np.array([[0,0]])

for topic, msg, t in bag_data.read_messages(topics=['/ardrone/bottom/image_raw', '/vicon/ARDroneCarre/ARDroneCarre']):
    if topic == '/vicon/ARDroneCarre/ARDroneCarre':
        curr_x = msg.transform.translation.x
        curr_y = msg.transform.translation.y
        curr_z = msg.transform.translation.z

    if topic == '/ardrone/bottom/image_raw':
        try:
            img = bridge.imgmsg_to_cv2(msg, "passthrough")
            rows, cols, channels = img.shape

            # Undistort the image
            img = cv.undistort(img, K, d)
            ## Smooth it
            img = cv.medianBlur(img,3)
            # Convert to greyscale
            img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
            # Apply Hough transform to greyscale image
            circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,param1=60,param2=40,minRadius=0,maxRadius=150)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                # Draw the circles
                for i in circles[0,:]:
                    # Print the coordinates of the circles
                    # draw the outer circle
                    cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                    # draw the center of the circle
                    cv.circle(img,(i[0],i[1]),2,(0,0,255),3)            
                    # converting from pixel coordinates to normalized coordinates
                    i_n = np.array(np.dot(np.linalg.inv(K),np.array([i[0],i[1],1])))
                    i_n = np.append(i_n,1)
                    # converting normalized coordinates to camera coordinates
                    i_c = np.array(np.dot(np.linalg.inv(T_cb), i_n.transpose()))
                    i_c = i_c.transpose()                    
                    real_coordinates = np.array([curr_x + i_c[0]*curr_z, curr_y + i_c[1]*curr_z])
                    #real_coordinates = real_coordinates.reshape(1,2)
                    cv.putText(img, str(real_coordinates), (i[0],i[1]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 3)
                    #all_coordinates = np.append(all_coordinates, real_coordinates, axis=0)
                    print(real_coordinates)

                # if(real_coordinates[0]>0 and real_coordinates[1]<0):
                #     C1 = np.append(C1,real_coordinates.reshape(1,2),axis=0)
                # elif(real_coordinates[0]<=1.9 and real_coordinates[0]>=1.5 and real_coordinates[1]<=0.7 and real_coordinates[1]>0.3):
                #     C2 = np.append(C2,real_coordinates.reshape(1,2),axis=0)
                # elif(real_coordinates[0]<1.5 and real_coordinates[0]>=1.1 and real_coordinates[1]<=1.5 and real_coordinates[1]>1.1):
                #     C3 = np.append(C3,real_coordinates.reshape(1,2),axis=0)
                # elif(real_coordinates[0]<0 and real_coordinates[1]>0):
                #     C4 = np.append(C4,real_coordinates.reshape(1,2),axis=0)                    
                # elif(real_coordinates[0]<=-0.3 and real_coordinates[0]>=-0.7 and real_coordinates[1]<=0 and real_coordinates[1]>=-0.4):
                #     C5 = np.append(C5,real_coordinates.reshape(1,2),axis=0)
                # elif(real_coordinates[0]<=-1.0 and real_coordinates[0]>=-1.4 and real_coordinates[1]<=-1.2 and real_coordinates[1]>=-1.6):
                #     C6 = np.append(C6,real_coordinates.reshape(1,2),axis=0)

            cv.imshow("Image window", img)
            # cv.imwrite('image_' + str(i) + '.jpg' , img)
            cv.waitKey(1000)
            # circle_num = 1
            # val = int(input())
            # input()
            # if val == 2:
            #     cv.imwrite('image.jpg' , img)

        except CvBridgeError as e:
            print(e)

# print("C1: "+ str(np.average(C1[1:,:], axis=0)))
# print("C2: "+ str(np.average(C2[1:,:], axis=0)))
# print("C3: "+ str(np.average(C3[1:,:], axis=0)))
# print("C4: "+ str(np.average(C4[1:,:], axis=0)))
# print("C5: "+ str(np.average(C5[1:,:], axis=0)))
# print("C6: "+ str(np.average(C6[1:,:], axis=0)))