from cv_bridge import CvBridge
import rosbag
import subprocess, yaml
import cv2 as cv
import numpy as np
import time
import math
from tf.transformations import euler_from_quaternion

# Intrinsic matrix
K = np.array([[604.62, 0.0, 320.5], [0.0, 604.62, 180.5], [0.0, 0.0, 1.0]])

## Tried implementing Brute Force Matcher but it was too slow and inaccurate - did not work properly.
# Implementing FLANN Based Matcher
MIN_MATCH_COUNT = 100
FLANN_INDEX_KDTREE = 1

input_img = str('princes_gates.png')
landmark_img = cv.imread(input_img)
detector = cv.xfeatures2d.SIFT_create()
kp_query, des_query = detector.detectAndCompute(landmark_img, None)

index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv.FlannBasedMatcher(index_params, search_params)

# counter for matched images
iter = 0

# landmark detection.
def landmarkDetection(msg):

    # Initialization
    good_matches = False
    bridge = CvBridge()
    dst = []

    # Converting ROS image to opencv image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    cv_image_copy = cv_image.copy()

    # Landmark detection
    kp_train, des_train = detector.detectAndCompute(cv_image_copy, None)

    if len(kp_train) > 1:

        matches = flann.knnMatch(des_query, des_train, k = 2)
        good = []
        
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([ kp_query[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp_train[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, maskk = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
            matchesMask = maskk.ravel().tolist()

            if M is not None:
                h,w,_ = landmark_img.shape
                pts = np.float32([ [0,0], [0,h-1], [w-1,h-1], [w-1,0], [w/2,h/2] ]).reshape(-1,1,2)
                dst = cv.perspectiveTransform(pts,M)

                if len(dst) != 0:
                    good_matches = True
                    cv_image_copy = cv.polylines(cv_image_copy,[np.int32(dst)],True,255,3, cv.LINE_AA)

                    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                                        singlePointColor = None,
                                        matchesMask = matchesMask, # draw only inliers
                                        flags = 2)
                    img3 = cv.drawMatches(landmark_img,kp_query,cv_image_copy,kp_train,good,None,**draw_params)
                    cv.imwrite('test_images/img3_{}.png'.format(iter), img3)

                else:
                    print('Perspective transform resulted in an empty matrix')

            else:
                print('Homography resulted in an empty matrix')

        else:
            print( "Not enough matches found - {}/{}".format(len(good), MIN_MATCH_COUNT) )

    return good_matches, dst

# Read bag file for messages.
bag = rosbag.Bag('FinalProject.bag')

# Read summary of bag file to test.
# This is a dictionary of all the data.
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', 'FinalProject.bag'], stdout=subprocess.PIPE).communicate()[0])

# Create variable based on number of images
num_images = info_dict["topics"][0]["messages"]
num = num_images

# Variable for appending topic data
pt_1 = []
pt_2 = []
pt_3 = []
pt_4 = []
landmarks_orientation = []
landmarks_location = []

# Transformation matrix
Tcb = np.array([[0.0, -1.0, 0.0, 0.0],[-1.0, 0.0, 0.0, 0.0125],[0.0, 0.0, -1.0, -0.025],[0.0, 0.0, 0.0, 1.0]])

for topic, msg, t in bag.read_messages(topics=['/vicon/ARDroneCarre/ARDroneCarre','/ardrone/bottom/image_raw']):

    if (topic=='/ardrone/bottom/image_raw'):

        iter += 1

        prev_quat = np.array([previous_msg.transform.rotation.x, previous_msg.transform.rotation.y, previous_msg.transform.rotation.z, previous_msg.transform.rotation.w])
        prev_euler = euler_from_quaternion(prev_quat)

        matches_flag, dst_points = landmarkDetection(msg)

        if matches_flag:
            
            # print([previous_msg.transform.translation.x, previous_msg.transform.translation.y, previous_msg.transform.translation.z])
            # Detecting position of the landmark
            x_cp = dst_points[4][0][0]
            y_cp = dst_points[4][0][1]

            xNyN_l = np.dot(np.linalg.inv(K), [x_cp,y_cp,1])

            x_l = xNyN_l[0]*previous_msg.transform.translation.z
            y_l = xNyN_l[1]*previous_msg.transform.translation.z

            target_l = np.dot(np.linalg.inv(Tcb),[x_l,y_l,previous_msg.transform.translation.z,1])

            result_l = np.add(np.array([target_l[0],target_l[1],target_l[2]]),np.array([previous_msg.transform.translation.x,previous_msg.transform.translation.y,
                            previous_msg.transform.translation.z]))

            landmarks_location.append(result_l)

            # Detecting pose of the landmark
            local_iter = 0

            for point in dst_points:

                local_iter += 1

                x_cp = point[0][0]
                y_cp = point[0][1]

                xNyN_l = np.dot(np.linalg.inv(K), [x_cp,y_cp,1])

                x_l = xNyN_l[0]*previous_msg.transform.translation.z
                y_l = xNyN_l[1]*previous_msg.transform.translation.z

                target_l = np.dot(np.linalg.inv(Tcb),[x_l,y_l,previous_msg.transform.translation.z,1])

                result_l = np.add(np.array([target_l[0],target_l[1],target_l[2]]),np.array([previous_msg.transform.translation.x,previous_msg.transform.translation.y,
                                previous_msg.transform.translation.z]))

                if local_iter == 1:
                    pt_1.append(result_l)
                elif local_iter == 2:
                    pt_2.append(result_l)
                elif local_iter == 3:
                    pt_3.append(result_l)
                elif local_iter == 4:
                    pt_4.append(result_l)

            # Prince's Gates and Casa Loma
            if pt_3[0][0] < pt_4[0][0] and pt_3[0][1] < pt_4[0][1]:
                alpha = math.atan((pt_4[0][1] - pt_3[0][1])/(pt_4[0][0]  - pt_3[0][0]))
                pose = alpha - math.radians(90 - math.degrees(prev_euler[2]))

            # CN Tower
            elif pt_3[0][0] < pt_4[0][0] and pt_3[0][1] > pt_4[0][1]:
                alpha = math.atan((pt_4[0][1] - pt_3[0][1])/(pt_4[0][0]  - pt_3[0][0]))
                pose = alpha - math.radians(90 - math.degrees(prev_euler[2]))

            # Nathan Philips Square
            elif pt_3[0][0] > pt_4[0][0] and pt_3[0][1] > pt_4[0][1]:
                alpha = math.atan((pt_4[0][1] - pt_3[0][1])/(pt_4[0][0] - pt_3[0][0]))
                pose = math.radians(- 90 - 90 -(90 - math.degrees(alpha + prev_euler[2])))

            landmarks_orientation.append(pose)

            pt_1 = []
            pt_2 = []
            pt_3 = []
            pt_4 = []

    previous_msg = msg
    previous_topic = topic

landmark_pos = sum(landmarks_location)/len(landmarks_location)
landmark_pose = sum(landmarks_orientation)/len(landmarks_orientation)

print(landmark_pos)
print(landmark_pose)