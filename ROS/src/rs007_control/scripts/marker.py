import rospy
from visualization_msgs.msg import Marker, MarkerArray


topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('register')

markerArray = MarkerArray()
count  = 0
MARKERS_MAX = 10

       # ... here I get the data I want to plot into a vector called trans
       
marker = Marker()
marker.header.frame_id = "/neck"
marker.type = marker.SPHERE
marker.action = marker.ADD
marker.scale.x = 0.2
marker.scale.y = 0.2
marker.scale.z = 0.2
marker.color.a = 1.0
marker.pose.orientation.w = 1.0
marker.pose.position.x = 0.5
marker.pose.position.y =  -0.2
marker.pose.position.z = 0.2 
       
markerArray.markers.append(marker)


# Publish the MarkerArray
publisher.publish(markerArray)
