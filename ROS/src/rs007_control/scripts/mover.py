#!/usr/bin/env python

# Mover1 - 23 April 2022

from __future__ import print_function

import rospy
from typing import Tuple

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveGroupCommander
from moveit_commander import RobotTrajectory


from rs007_control.srv import MoverService, MoverServiceRequest, MoverServiceResponse
print("loading rs007_control:mover.py")

joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
print("joint_names",joint_names)

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan: Tuple[int,RobotTrajectory,float,int]) -> RobotTrajectory:
        # Note - here are all the values
        # return (
        #     error_code.val == MoveItErrorCodes.SUCCESS,
        #     plan.deserialize(trajectory_msg),
        #     planning_time,
        #     error_code,
        # )        
        return plan[1]
else:
    def planCompat(plan : RobotTrajectory) -> RobotTrajectory:
        return plan
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group:MoveGroupCommander, destination_pose:Pose, 
                   start_joint_angles:Tuple[float,float,float,float,float,float]) -> RobotTrajectory:  
    print("rs007_control:mover.py:plan_trajectory")
    print("======================================")
    # print("    move_group:",move_group)
    # print("    type(destination_pose):",type(destination_pose))
    print("destination_pose:",destination_pose)
    # print("    type(start_joint_angles):",type(start_joint_angles))
    print("start_joint_angles:",start_joint_angles)
    print("--------------------------------------------")
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles
    # print("    current_joint_state:",current_joint_state)
    
    moveit_robot_state = RobotState()
    # print("    moveit_robot_state:",moveit_robot_state)
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)
    
    move_group.set_pose_target(destination_pose)
    print("    calculating plan")   
    plan = move_group.plan()
    # print("    plan:",plan)   
    

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)
        
    print(" plan_trajectory - returning planCompat(plan)")   
    return planCompat(plan)

def cartesian_plan(move_group:MoveGroupCommander,tu_pose:Pose,fr_pose:Pose,
                   current_joints:Tuple[float,float,float,float,float,float],nbetween=2):
    print("cartesian_plan")
    print("==============")
    # print("type(move_group)",type(move_group))
    # print("move_group")
    # print(move_group)
    # print("type(tu_pose)",type(tu_pose))
    print("tu_pose")
    print(tu_pose)
    # print("type(fr_pose)",type(fr_pose))
    print("fr_pose")
    print(fr_pose)
    # print("type(current_joints)",type(current_joints))
    print(current_joints)    
    print("nbetween:"+str(nbetween))

    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = current_joints
    # print("    current_joint_state:",current_joint_state)
 

    moveit_robot_state = RobotState()
    # print("    moveit_robot_state:",moveit_robot_state)
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)
    move_group.set_pose_target(tu_pose)    

    waypoints = []
    
    waypoints.append(copy.deepcopy(fr_pose))
    
    deltx = tu_pose.position.x-fr_pose.position.x
    delty = tu_pose.position.y-fr_pose.position.y
    deltz = tu_pose.position.z-fr_pose.position.z

  
    for i in range(0,nbetween):
        frac = (i+1.0)/(nbetween+1.0)
        wpose = fr_pose
        wx = deltx*frac + fr_pose.position.x
        wy = delty*frac + fr_pose.position.y
        wz = deltz*frac + fr_pose.position.z
        wpose.position.x = wx
        wpose.position.y = wy
        wpose.position.z = wz
        wpose.orientation.x = 0
        wpose.orientation.y = -1
        wpose.orientation.z = 0
        wpose.orientation.w = 0
        # print("waypoint:"+str(i+1)+" wx"+str(wx)+" wy:"+str(wy)+" wz:"+str(wz))        
        waypoints.append(copy.deepcopy(wpose))

    waypoints.append(copy.deepcopy(tu_pose))
    
    eef_step_in_meters = 0.01
    jump_threshold = 0

    (plan,fraction) = move_group.compute_cartesian_path(waypoints, eef_step_in_meters, jump_threshold )
    print("cartesian_plan finished - returning plan")
    # print(plan)
    return plan
    
"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""


def plan_pose_sequence(req:MoverServiceRequest):
    print("rs007_control:mover.py:plan_pose_sequence")
    print("=========================================")
    
    # print("    type(req):",type(req))
    # print("    req:",req)
    response = MoverServiceResponse()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    i = 0
    for pose in req.joints_input.poses:
        print(f"pose[{i}]:\n{pose}")
        i += 1


    # Pre grasp - position gripper directly above target object
    curpose = req.joints_input.poses[0]
    trajpt = plan_trajectory(move_group, curpose, current_robot_joint_configuration)
    response.trajectories.append(trajpt)
    prev_angles = trajpt.joint_trajectory.points[-1].positions

    nposes = len(req.joints_input.poses)
    for i in range(1,nposes):
        pose = req.joints_input.poses[i]
        trajpt = cartesian_plan(move_group, pose, curpose, prev_angles)
        if not trajpt.joint_trajectory.points:
            return response        
        response.trajectories.append(trajpt)
        prev_angles = trajpt.joint_trajectory.points[-1].positions
        curpose = pose
   
    # If the trajectory has no points, planning has failed and we return an empty response
    move_group.clear_pose_targets()

    return response

def plan_pose_sequence_old(req:MoverServiceRequest):
    print("rs007_control:mover.py:plan_pose_sequence")
    print("=========================================")
    
    print("    type(req):",type(req))
    print("    req:",req)
    response = MoverServiceResponse()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    req_pick_pose = req.joints_input.poses[0]
    print("req_pick_pose",req_pick_pose)
    grasp_pick_pose = req.joints_input.poses[1]
    print("grasp_pick_pose",grasp_pick_pose)
    # grasp_pick_pose = copy.deepcopy(req.pick_pose)
    # grasp_pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    req_place_pose = req.joints_input.poses[2]
    print("req_place_pose",req_place_pose)

    # Pre grasp - position gripper directly above target object
    pre_grasp_trajpt = plan_trajectory(move_group, req_pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_trajpt.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_trajpt.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    
    # grasp_pose = None
    grasp_trajpt = cartesian_plan(move_group, grasp_pick_pose, req_pick_pose, previous_ending_joint_angles)
       

    previous_ending_joint_angles = grasp_trajpt.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    
    pick_up_trajpt = cartesian_plan(move_group, req_pick_pose, grasp_pick_pose, previous_ending_joint_angles)
    
    if not pick_up_trajpt.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_trajpt.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    place_trajpt = cartesian_plan(move_group,  req_place_pose, req_pick_pose, previous_ending_joint_angles)

    if not place_trajpt.joint_trajectory.points:
        return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_trajpt)
    response.trajectories.append(grasp_trajpt)
    response.trajectories.append(pick_up_trajpt)
    response.trajectories.append(place_trajpt)

    move_group.clear_pose_targets()

    return response    


def plan_pick_and_place(req:MoverServiceRequest):
    print("rs007_control:mover.py:plan_pick_and_place")
    print("==========================================")
   
    print("    type(req):",type(req))
    print("    req:",req)
    response = MoverServiceResponse()


    group_name = "manipulator"
    move_group = MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

    # Grasp - lower gripper so that fingers are on either side of object
    orig_pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    # print("current_robot_joint_configuration")
    # print(current_robot_joint_configuration)   
    # print("previous_ending_joint_angles")
    # print(previous_ending_joint_angles)
    
    # grasp_pose = None
    grasp_pose = cartesian_plan(move_group, pick_pose, req.pick_pose, previous_ending_joint_angles,)
        

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    #pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
    pick_up_pose = cartesian_plan(move_group, orig_pick_pose, pick_pose, previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return response

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    # place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)
    place_pose = cartesian_plan(move_group,  req.place_pose, orig_pick_pose, previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return response

    # If trajectory planning worked for all pick and place stages, add plan to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_pose)
    response.trajectories.append(pick_up_pose)
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response




def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('rs007_moveit_server')

    s1 = rospy.Service('rs007_moveit', MoverService, plan_pick_and_place)
    s2 = rospy.Service('rs007_moveit_poseseq', MoverService, plan_pose_sequence)
                        
    print("rs007_control:mover.py:Ready to plan - with poseseq - spinning - ver 29 June 2022 - 14:33")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
