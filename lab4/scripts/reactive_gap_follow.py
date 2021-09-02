#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100) 
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges = np.array(ranges)
        max_value = 5
        window_size = 7

        # filter inf, NaN & element > max_value
        idx = (~np.isfinite(ranges)) | (ranges > max_value)
        ranges[idx] = max_value

        # 1D convolution
        kernel = np.ones(window_size)
        proc_ranges = np.convolve(ranges, kernel, 'same')/window_size

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        gaps = np.split(free_space_ranges, np.where(free_space_ranges== 0.)[0][1:])
        gaps_length = np.array(list(map(lambda x: len(x), gaps)))
        idx = np.argmax(gaps_length)
        init = np.sum(gaps_length[:idx]) + 1
        return init, init+gaps_length[idx]-2
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        return start_i + np.argmax(ranges[start_i:end_i+1])

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        ranges = data.ranges
        ranges = self.preprocess_lidar(ranges)
        threshold = 0.5

        #Find closest point to LiDAR
        idx = np.argmin(ranges)
        d = ranges[idx]
        angle = angles[idx]

        #Eliminate all points inside 'bubble' (set them to zero) 
        distances = ((ranges*np.cos(angles) - d*np.cos(angle))**2 + (ranges*np.sin(angles) - d*np.sin(angle))**2)**0.5
        distances[distances<threshold] = 0

        #Find max length gap 
        start, end = self.find_max_gap(distances)

        #Find the best point in the gap 
        best_pt = self.find_best_point(start, end, ranges)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = rospy.Time.now()
        # drive_msg.header.frame_id = "laser"
        drive_msg.drive.speed = 1.0
        drive_msg.drive.steering_angle = angles[best_pt]
        self.drive_pub.publish(drive_msg)

        # print(f'Steering towards: {angles[best_pt]}')



def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)