#!/usr/bin/env python

import rospy

from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose

if __name__ == '__main__':

    #init_node()
    rospy.init_node('message', anonymous=True)
    group = MoveGroupCommander("manipulator")
    psi = PlanningSceneInterface()
    exec_vel = 0.5
    print(f"get_goal_joint_tolerance:{group.get_goal_joint_tolerance()}")
    print(f"=====================================================================")

    name = "sphere1"
    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = -0.2
    pose.position.z = 0.2
    psi.add_sphere(name,pose)

