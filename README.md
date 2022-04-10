<p align="center"><img src="img/0_pick_place.gif"/></p>

# Kawaski Heavy Industry Pick-and-Place 

Derived from the Unity Robotics Hub Pick-and-Place tutorial

# Changes:
- Unity
 - Added scene with khi rs007n robot
 - Added scene with flat plane to accomodate the larger khi robots rs007n and rs007l
 - Added gripper from niryo_one robot to rs007n robot
 
# Docker
 - Added khi_robot packages to ROS packages
 - Added bat files to make docker builds
 - Added docker build for noetic
 - Added docker build for noetic and melodic with khi_robot packages
 
# Robot control
 - Got moveit control to work with khi rs007n robot but not correctly (using internal niryo_one geometry)

# New Repo without submodiles