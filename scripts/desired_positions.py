#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
from std_msgs.msg import String

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator

class ROSDesiredPositionGenerator(object):

    def __init__(self):

        """Initialize the ROSDesiredPositionGenerator class."""

        # Publisher
        self.pub_des_pos = rospy.Publisher('des_pos', String, queue_size=10)
        
        # Run the onboard controller at 200 Hz.
        self.onboard_loop_frequency = 200.

        self.current_point = 1
        
        # Run this ROS node at the onboard loop frequency.
        self.run_pub_des_pos = rospy.Timer(rospy.Duration(1. / 
            self.onboard_loop_frequency), self.send_des_pos)


        # Linear trajectory setup.

        linear_start = np.array([[-1.5, -1.5, 1, 0]])
        linear_end = np.array([[1.5, 1.5, 2, 0]])

        # Number of intermediate points
        linear_intpoints = 6

        # Placeholder for intermediate points.
        linear_midpoints = np.ones((linear_intpoints, 4))

        # Calculate each intermediate point required (one way).
        for i in range(1, linear_intpoints+1):
            linear_midpoints[i-1,:] = linear_start + (i/(linear_intpoints+1))*((linear_end) - (linear_start))

        # Concatenating the trajectory together. 
        # Flipping the intermediate points for reverse direction.

        self.linear_trajectory = np.concatenate((linear_start, 
                                            linear_midpoints, 
                                            linear_end, 
                                            np.flipud(linear_midpoints), 
                                            linear_start), axis = 0)


        # Setting up circular trajectory.

        circular_origin = np.array([[0, 0,0.5, -np.pi]])
        circular_height = 1  
        circular_radius = 1.5

        # Number of circular waypoints points. # Please keep this number even.
        circular_waypoints = 12

        # Placeholder for circular trajectory points.
        self.circular_trajectory = np.ones((circular_waypoints+1, 4))

        # Calculate each circular height points.
        circular_height_pts_half = np.linspace(circular_origin[0,2], circular_origin[0,2] + circular_height*(1 - 1/circular_waypoints), circular_waypoints/2)
        circular_height_pts_other_half = np.linspace(circular_origin[0,2] + circular_height, circular_origin[0,2], 1 + 0.5*circular_waypoints)
        circular_height_pts = np.concatenate((circular_height_pts_half, circular_height_pts_other_half), axis = 0)
        print(np.size(circular_height_pts))

        # Calculate each circular trajectory point required (one cycle).
        for i in range(circular_waypoints +1):
            angle_segment = i*((2*np.pi)/circular_waypoints)
            self.circular_trajectory[i,:] = [circular_origin[0,0] + circular_radius*np.cos(angle_segment), 
                                        circular_origin[0,1] + circular_radius*np.sin(angle_segment), 
                                        circular_height_pts[i],
                                        angle_segment - np.pi]

    #---------------------------------------------------------------------
        """ZIGZAG TRAJECTORY"""  
    #---------------------------------------------------------------------


        # Setting up zigzag trajectory.
        zigzag_intpoints = 6
        scale_factor = 0.75;
        zigzag_start = scale_factor*np.array([[-2, 2, 2, 0]])  
        zigzag_end = scale_factor*np.array([[-2, -2, 2, 0]])
        zigzag_waypoints_1 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_1[i-1, :] = zigzag_start + i*((zigzag_end) - (zigzag_start))/(zigzag_intpoints+1)

        zigzag_start_2 = zigzag_end 
        zigzag_end_2 = scale_factor*np.array([[-1, -2, 2, 0]])
        zigzag_waypoints_2 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_2[i-1, :] = zigzag_start_2 + i*((zigzag_end_2) - (zigzag_start_2))/(zigzag_intpoints+1)

        zigzag_start_3 = zigzag_end_2
        zigzag_end_3 = scale_factor*np.array([[-1, 2, 2, 0]])
        zigzag_waypoints_3 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_3[i-1, :] = zigzag_start_3 + i*((zigzag_end_3) - (zigzag_start_3))/(zigzag_intpoints+1)

        zigzag_start_4 = zigzag_end_3 
        zigzag_end_4 = scale_factor*np.array([[0, 2, 2, 0]])
        zigzag_waypoints_4 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_4[i-1, :] = zigzag_start_4 + i*((zigzag_end_4) - (zigzag_start_4))/(zigzag_intpoints+1)

        zigzag_start_5 = zigzag_end_4 
        zigzag_end_5 = scale_factor*np.array([[0, -2, 2, 0]])
        zigzag_waypoints_5 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_5[i-1, :] = zigzag_start_5 + i*((zigzag_end_5) - (zigzag_start_5))/(zigzag_intpoints+1)

        zigzag_start_6 = zigzag_end_5 
        zigzag_end_6 = scale_factor*np.array([[1, -2, 2, 0]])
        zigzag_waypoints_6 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_6[i-1, :] = zigzag_start_6 + i*((zigzag_end_6) - (zigzag_start_6))/(zigzag_intpoints+1)

        zigzag_start_7 = zigzag_end_6
        zigzag_end_7 = scale_factor*np.array([[1, 2, 2, 0]])
        zigzag_waypoints_7 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_7[i-1, :] = zigzag_start_7 + i*((zigzag_end_7) - (zigzag_start_7))/(zigzag_intpoints+1)

        zigzag_start_8 = zigzag_end_7
        zigzag_end_8 = scale_factor*np.array([[2, 2, 2, 0]])
        zigzag_waypoints_8 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_8[i-1, :] = zigzag_start_8 + i*((zigzag_end_8) - (zigzag_start_8))/(zigzag_intpoints+1)

        zigzag_start_9 = zigzag_end_8 
        zigzag_end_9 = scale_factor*np.array([[2, -2, 2, 0]])
        zigzag_waypoints_9 = np.ones((zigzag_intpoints, 4))
        for i in range(1, zigzag_intpoints+1):
            zigzag_waypoints_9[i-1, :] = zigzag_start_9 + i*((zigzag_end_9) - (zigzag_start_9))/(zigzag_intpoints+1)

        self.zigzag_trajectory = np.concatenate((zigzag_start, zigzag_waypoints_1, zigzag_end, zigzag_start_2, zigzag_waypoints_2, zigzag_end_2, zigzag_start_3, zigzag_waypoints_3, 
                           zigzag_end_3, zigzag_start_4, zigzag_waypoints_4, zigzag_end_4, zigzag_start_5, zigzag_waypoints_5, zigzag_end_5, zigzag_start_6, zigzag_waypoints_6, zigzag_end_6,
                           zigzag_start_7, zigzag_waypoints_7, zigzag_end_7, zigzag_start_8, zigzag_waypoints_8, zigzag_end_8, zigzag_start_9, zigzag_waypoints_9, zigzag_end_9), axis = 0)


    def send_des_pos(self, event):
        """Publish the entire trajectory as a 1D string using the existing String msg type."""
        
        # Creating a 1D flattened array of desired_trajectory.
        msg_array = self.zigzag_trajectory.flatten()

        # Converting the 1D msg array to a string and getting rid of brackets. 
        self.msg_string = ' '.join(map(str, msg_array))

        # Publishing the desired trajectory msg to des_pos.
        self.pub_des_pos.publish(self.msg_string)

if __name__ == '__main__':
    rospy.init_node('desired_position')
    ROSDesiredPositionGenerator()
    rospy.spin()
