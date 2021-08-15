#!/usr/bin/env python

import rospy 
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64 

from ychau001_lab1.msg import scan_range
from lidar_preprocessing import preprocess_lidar

class Echo:

    def __init__(self):
        self.data = None 

        rospy.init_node('echo', anonymous=True)
        
        #publishes to three diff topic
        self.closest_pub = rospy.Publisher('/closest_point', Float64, queue_size=1000)
        self.farthest_pub = rospy.Publisher('/farthest_point', Float64, queue_size=1000)
        self.scan_range_pub = rospy.Publisher('/scan_range', scan_range, queue_size=1000)  

        #subscribes to scan
        rospy.Subscriber('scan', LaserScan, self.callback)
        timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback) #publish at a fixed time interval, 0.5s
        rospy.spin()
        timer.shutdown()
    
    def callback(self, data):
        print("New message received")
        self.data = preprocess_lidar(data.ranges) 

    def timer_callback(self, event):
 
        min = np.min(self.data)
        max = np.max(self.data)

        self.closest_pub.publish(min)
        print("Last message published to closest_point")
        self.farthest_pub.publish(max)
        print("Last message published to farthest_point")

        msg = scan_range()
        msg.range_min = min
        msg.range_max = max 
        self.scan_range_pub.publish(msg)
        print("Last message published to scan_range")

if __name__ == "__main__":
    print("Hello World")
    Echo()
