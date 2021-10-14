#!/usr/bin/env python3

import tf
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped

from visualizer import plot_marker


rospack = rospkg.RosPack()
package_path = rospack.get_path('lab6')

class PurePursuit(object):
	"""
	The class that handles pure pursuit.
	"""
	def __init__(self):
		#subs & pubs
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size = 1)
		self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1)
		self.marker_pub = rospy.Publisher('/goal_point', Marker, queue_size = 1)

		self.l = 0.5
		self.waypoints = np.genfromtxt(package_path+'/logs/waypoints.csv', delimiter=',')[:, :2]
		self.listener = tf.TransformListener()

	def pose_callback(self, data):
		#get pose of car
		orient = data.pose.pose.orientation 
		pos = data.pose.pose.position

		#construct transformation matrix
		matrix = np.zeros((4,4))
		rot = np.array([orient.x, orient.y, orient.z, orient.w])
		rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
		trans = np.array([pos.x, pos.y, pos.z])
		matrix[:3,:3] = rot
		matrix[:3, 3] = trans
		matrix[-1, -1] = 1 

		#construct x,y,z,k input 
		n = len(self.waypoints)
		ipt = np.zeros((4, n))
		ipt[:2, :] = self.waypoints.T
		ipt[3, :] = 1

		#transform to base link (car's frame)
		opt = np.linalg.inv(matrix).dot(ipt)
		xy = opt[:2, :].T #transformed
		xy[xy[:,0]<0] = 10 #filter points behind the car

		#select goal point
		distance = np.sum(xy**2, axis=1)
		idx = np.argmin(np.absolute(distance-self.l**2))
		goal_x, goal_y = xy[idx]
		plot_marker(self.marker_pub, goal_x, goal_y) #visualize goal point

		#steer
		steering_angle = 2 * goal_y / self.l ** 2
		self.steer(steering_angle)
	
	def steer(self, angle): 	
		if -np.pi/18 < angle < np.pi/18:
			velocity = 3
		elif -np.pi/9 < angle <= -np.pi/18 or np.pi/18 <= angle < np.pi/9:
			velocity = 2
		else:
			velocity = 1
			
		drive_msg = AckermannDriveStamped()
		drive_msg.header.stamp = rospy.Time.now()
		drive_msg.header.frame_id = "laser"
		drive_msg.drive.steering_angle = angle
		drive_msg.drive.speed = velocity
		self.drive_pub.publish(drive_msg)					
	
def main():
	rospy.init_node('pure_pursuit_node')
	pp = PurePursuit()
	rospy.spin()

if __name__ == '__main__':
	main()
