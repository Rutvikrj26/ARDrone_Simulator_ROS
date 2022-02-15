#!/usr/bin/env python

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

import rosbag

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from position_controller import PositionController

from std_msgs.msg import String, Empty

class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here to define node publishers and subscribers

    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback
    #pass
    def __init__(self):

        # Publishers
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size= 1) # land the drone
        self.pub_traj = rospy.Publisher('/cmd_vel_RHC', Twist, queue_size = 32) # publish the Velocity Data         
        self.pub_plot_curr = rospy.Publisher('plot_data_curr', Twist, queue_size=30) # publish the current position
        self.pub_plot_des = rospy.Publisher('plot_data_des', Twist, queue_size=30) # publish the desired position
        self.pub_plot_err = rospy.Publisher('plot_data_err', Twist, queue_size=30) # publish the error

        # Subscribers
        self.vicon_msg = TransformStamped()
        self.model_name = 'ARDroneCarre'
        
        self.sub_vicon_data = rospy.Subscriber('/vicon/{0}/{0}'.format(self.model_name),TransformStamped, self.update_vicon_data) # subscribe to the vicon data
        self.sub_des_pos = rospy.Subscriber('des_pos', String, self.update_desired_position) # subscribe to the desired position

        # Initialize messages for publishing
        self.cmd_vel_msg = Twist()
        self.plot_curr_msg = Twist()
        self.plot_des_msg = Twist()
        self.plot_err_msg = Twist()

        # Run the onboard controller at 200 Hz
        self.onboard_loop_frequency = 200.
        
        # Calling the position controller to pass the data
        self.pos_class = PositionController()

        # Run this ROS node at the onboard loop frequency
        self.run_pub_cmd_vel = rospy.Timer(rospy.Duration(1. / self.onboard_loop_frequency), self.update_roll_pitch_yaw)
        self.run_pub_plot_curr = rospy.Timer(rospy.Duration(1. / self.onboard_loop_frequency), self.plot_curr)
        self.run_pub_plot_des = rospy.Timer(rospy.Duration(1. / self.onboard_loop_frequency), self.plot_des)
        self.run_pub_plot_err = rospy.Timer(rospy.Duration(1. / self.onboard_loop_frequency), self.plot_err)
        self.current_point = 1
        self.nonunix_time = 0
        self.dt = 0

        self.num_points_old = 3

        # Keep time for differentiation and integration within the controller
        self.old_time = rospy.get_time()
        self.start_time = rospy.get_time()

    def land(self):
        msg = Empty()
        self.pub_land.publish(msg)

    def update_roll_pitch_yaw(self, event):
        """Determine the motor speeds and and publishes these."""
        
        # Determine the time step for differentiation and integration
        current_time = rospy.get_time()
        self.dt = current_time - self.old_time
        
        # Get the next set of postioning commands from the position controller
        new_controls = self.pos_class.update_position_controller(self.dt)

        # Rearranging the data for ease
        [new_roll, new_pitch, new_yaw_rate, new_climb_rate] = new_controls
        
        # Setting the cmd_vel msg values to the desired ones
        self.cmd_vel_msg.linear.x = new_roll
        self.cmd_vel_msg.linear.y = new_pitch
        self.cmd_vel_msg.angular.z = new_yaw_rate
        self.cmd_vel_msg.linear.z = new_climb_rate
        
        # Publish the motor commands for the ardrone plugin
        self.pub_traj.publish(self.cmd_vel_msg)
        
        # Set the old time to the current for future time step calculations
        self.old_time = current_time


    def update_vicon_data(self, vicon_data_msg):

        # Updating the Translational data from the vicon message
        (self.pos_class.current_trans_x,
        self.pos_class.current_trans_y,
        self.pos_class.current_trans_z) = (vicon_data_msg.transform.translation.x,
             vicon_data_msg.transform.translation.y,
             vicon_data_msg.transform.translation.z)
        
        # Updating the Rotational data from the vicon message
        (self.pos_class.current_rot_x,
        self.pos_class.current_rot_y,
        self.pos_class.current_rot_z,
        self.pos_class.current_rot_w) = (vicon_data_msg.transform.rotation.x,
             vicon_data_msg.transform.rotation.y,
             vicon_data_msg.transform.rotation.z,
             vicon_data_msg.transform.rotation.w)

    def update_desired_position(self, pos_msg):

        # Pre-Processing the incoming Desired Position messages    
        self.des_pos_msg = np.fromstring(pos_msg.data, dtype = float, sep = ' ')

        num_points = self.des_pos_msg.size/4
        if num_points != self.num_points_old:
            self.num_points_old = num_points
            self.current_point = 1

        trajectory = np.reshape(self.des_pos_msg, (-1, 4))

        # Updating the Desired Position data from the desired position message
        self.pos_class.desired_x = trajectory[self.current_point - 1, 0]
        self.pos_class.desired_y = trajectory[self.current_point - 1, 1]
        self.pos_class.desired_z = trajectory[self.current_point - 1, 2]
        self.pos_class.desired_yaw = trajectory[self.current_point - 1, 3]

        # Updating the time, based on which the desired position is being updated each time
        self.nonunix_time += self.dt
        if (((self.pos_class.desired_x - 0.10) < self.pos_class.current_trans_x < (self.pos_class.desired_x + 0.1)) and
            ((self.pos_class.desired_y - 0.1) < self.pos_class.current_trans_y < (self.pos_class.desired_y + 0.1)) and
            ((self.pos_class.desired_z - 0.05) < self.pos_class.current_trans_z < (self.pos_class.desired_z + 0.05)) and
            ((self.pos_class.desired_yaw - 0.15) < self.pos_class.current_yaw < (self.pos_class.desired_yaw + 0.15))):
            if self.current_point < num_points and self.nonunix_time >= 0.01:
                self.current_point += 1
                self.nonunix_time = 0

 
    # Function to publish the data to plot the current position
    def plot_curr(self, event):
        self.plot_curr_msg.linear.x = self.pos_class.current_trans_x
        self.plot_curr_msg.linear.y = self.pos_class.current_trans_y
        self.plot_curr_msg.linear.z = self.pos_class.current_trans_z
        self.plot_curr_msg.angular.z = self.pos_class.current_yaw

        self.pub_plot_curr.publish(self.plot_curr_msg)

    # Function to publish the data to plot the Desired position
    def plot_des(self, event):
        self.plot_des_msg.linear.x = self.pos_class.desired_x
        self.plot_des_msg.linear.y = self.pos_class.desired_y
        self.plot_des_msg.linear.z = self.pos_class.desired_z
        self.plot_des_msg.angular.z = self.pos_class.desired_yaw

        self.pub_plot_des.publish(self.plot_des_msg)

    # Function to publish the data to plot the position error
    def plot_err(self, event):
        self.plot_err_msg.linear.x = self.pos_class.x_err
        self.plot_err_msg.linear.y = self.pos_class.y_err
        self.plot_err_msg.linear.z = self.pos_class.z_err
        self.plot_err_msg.angular.z = self.pos_class.yaw_err

        self.pub_plot_err.publish(self.plot_err_msg)


if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('ros_interface', disable_signals = True)
    ardrone = ROSControllerNode()

    rospy.spin()