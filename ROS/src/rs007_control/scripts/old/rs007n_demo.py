#!/usr/bin/env python

import rospy

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':

    #init_node()
    rospy.init_node('message', anonymous=True)
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
        pass
