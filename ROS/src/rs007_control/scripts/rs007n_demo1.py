#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy

from moveit_commander import MoveGroupCommander

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

def pub_markers():
    topic = 'visualization_marker_array'
    global markcnt
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

    markerArray = MarkerArray()



    marker = get_marker(0,1,0, 0.5,-0.2,0.2)
    markerArray.markers.append(marker)

    marker = get_marker(0,0,1, 0.5,-0.2,0.7)
    markerArray.markers.append(marker)

    marker = get_marker(1,1,0, 0.5,0.1,0.7)
    markerArray.markers.append(marker)

    marker = get_marker(1,0,0, 0.5,0.1,0.2)
    markerArray.markers.append(marker)
    
    rospy.sleep(2)
    publisher.publish(markerArray)
    print(f"Published {markcnt} markers")



def animate_robot():
    group = MoveGroupCommander("manipulator")
    exec_vel = 0.5
    print(f"get_goal_joint_tolerance:{group.get_goal_joint_tolerance()}")
    print(f"get_planning_time:{group.get_planning_time()}")
    print(f"get_goal_tolerance:{group.get_goal_tolerance()}")
    print(f"get_goal_position_tolerance:{group.get_goal_position_tolerance()}")
    print(f"get_goal_orientation_tolerance:{group.get_goal_orientation_tolerance()}")
    print(f"=====================================================================")

    try:
        while True:

            rospy.loginfo("joint1 start")
            group.set_max_velocity_scaling_factor(exec_vel)
            group.set_joint_value_target([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
            group.go()
            rospy.loginfo("joint1 end")
            input("Press enter to contine")

            rospy.loginfo("pose1 start")
            group.set_max_velocity_scaling_factor(exec_vel)
            group.set_pose_target([0.5,-0.2,0.2,0.0,1.0,0.0])
            group.go()
            rospy.loginfo("pose1 end")
            input("Press enter to contine")

            rospy.loginfo("pose2 start")
            group.set_max_velocity_scaling_factor(exec_vel)
            group.set_pose_target([0.5,-0.2,0.7,0.0,1.0,0.0])
            group.go()
            rospy.loginfo("pose2 end")
            input("Press enter to contine")

            rospy.loginfo("pose3 start")
            group.set_max_velocity_scaling_factor(exec_vel)
            group.set_pose_target([0.5,0.1,0.7,0.0,1.0,0.0])
            group.go()
            rospy.loginfo("pose3 end")
            input("Press enter to contine")

            rospy.loginfo("pose4 start")
            group.set_max_velocity_scaling_factor(exec_vel)
            group.set_pose_target([0.5,0.1,0.2,0.0,1.0,0.0])
            group.go()
            rospy.loginfo("pose4 end")
            input("Press enter to contine")

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt exception caught")


rospy.init_node('rs007n_control_test', anonymous=True,log_level=rospy.INFO, disable_signals=False)
pub_markers()
animate_robot()