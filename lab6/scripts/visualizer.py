from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA

def plot_marker(publisher, goal_x, goal_y):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time()
    marker.type = 0 #shape
    marker.action = 0

    pos = Pose()
    pos.position.x  = goal_x
    pos.position.y = goal_y
    pos.position.z = 0
    pos.orientation.x = 0
    pos.orientation.y = 0
    pos.orientation.z = 0
    pos.orientation.w = 1
    marker.pose = pos

    s = Vector3() #scale
    s.x = 0.4
    s.y = 0.4
    s.z = 0.4
    marker.scale = s

    colour = ColorRGBA() #color
    colour.r = 255
    colour.g = 0
    colour.b = 0
    colour.a = 1
    marker.color = colour

    marker.lifetime.secs = 0
    marker.frame_locked = True
    publisher.publish(marker)