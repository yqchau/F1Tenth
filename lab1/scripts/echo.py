#!/usr/bin/env python

import rospy 
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64 

from ychau001_lab1.msg import scan_range
from lidar_processing import process_lidar

class Echo:

    def __init__(self):
        self.data = None 

        rospy.init_node('echo', anonymous=True)
        
        #publishes to three diff topic
        self.closest_pub = rospy.Publisher('closest_point', Float64, queue_size=1000)
        self.farthest_pub = rospy.Publisher('farthest_point', Float64, queue_size=1000)
        self.scan_range_pub = rospy.Publisher('scan_range', scan_range, queue_size=1000)  

        #subscribes to scan
        rospy.Subscriber('scan', LaserScan, self.callback)
    
    def callback(self, data):
        rospy.loginfo("New message received")
        self.data = process_lidar(data.ranges) 
 
        min = np.min(self.data)
        max = np.max(self.data)

        self.closest_pub.publish(min)
        rospy.loginfo("Message published to closest_point")

        self.farthest_pub.publish(max)
        rospy.loginfo("Message published to farthest_point")

        msg = scan_range()
        msg.range_min = min
        msg.range_max = max 
        self.scan_range_pub.publish(msg)
        rospy.loginfo("Message published to scan_range")


def main():
    echoer = Echo()
    rospy.spin()


if __name__ == "__main__":
    main()

