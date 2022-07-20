#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math


markcnt = 0
def get_marker(r,g,b,x,y,z):
    global markcnt
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = markcnt
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.04
    marker.scale.y = 0.04
    marker.scale.z = 0.04

    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    markcnt += 1
    return marker


topic = 'visualization_marker_array'

publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

rospy.init_node('register')

markerArray = MarkerArray()



marker = get_marker(0,1,0, 0.5,-0.2,0.2)
markerArray.markers.append(marker)

marker = get_marker(0,0,1, 0.5,-0.2,0.7)
markerArray.markers.append(marker)

marker = get_marker(1,1,0, 0.5,0.1,0.7)
markerArray.markers.append(marker)

marker = get_marker(1,0,0, 0.5,0.1,0.2)
markerArray.markers.append(marker)

print("turn on 4")
rospy.sleep(4.5)
print("publish")
publisher.publish(markerArray)

rospy.sleep(10.5)
print(f"marker count:{markcnt}")
     
