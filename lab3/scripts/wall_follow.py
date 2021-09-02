#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 8
kd = 0.00001
ki = 0.00001
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
DESIRED_DISTANCE_LEFT = 0.90
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

# OTHER PARAMS
PRE_T = 0.0 
CUR_T = 0.0

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100) 

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        idx = int((angle + 3.14) / data.angle_increment)
        return data.ranges[idx]

    def pid_control(self, error):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global CUR_T
        global PRE_T
        angle = 0.0

        # Use kp, ki & kd to implement a PID controller for 
        PRE_T = CUR_T
        CUR_T = rospy.get_time()
        dt = float(CUR_T - PRE_T)

        p_control_value = kp*error + kd*(error-prev_error)/dt + ki*(integral)
        integral += error

        # Publishing message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = p_control_value

        if (abs(p_control_value) > 0.349):
            drive_msg.drive.speed = 1.5
        elif (abs(p_control_value) > 0.174):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5

        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        # Follow left wall as per the algorithm 
        a_angle = 0.7853 # pi/4
        b_angle = 1.57063 # pi/2

        a = self.getRange(data, a_angle)
        b = self.getRange(data, b_angle)
        theta = 0.7853
        alpha = (a*np.cos(theta) - b)/(a*np.sin(theta))
        alpha = np.arctan(alpha)

        d_t = b * np.cos(alpha)
        d_tp1 = d_t + CAR_LENGTH*np.sin(alpha)

        return (d_tp1 - leftDist)

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        self.pid_control(error)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
