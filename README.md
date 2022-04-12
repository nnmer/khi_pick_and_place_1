<p align="center"><img src="img/0_pick_place.gif"/></p>

# Kawaski Heavy Industry Pick-and-Place 

Derived from the Unity Robotics Hub Pick-and-Place tutorial


# Getting started
- prerequisites Windows 10 or 11, Docker Desktop, Unity
- You will want to build the ROS Docker images 
   - we have scripts to build and run them for ROS1: Melodic and Noetic
   - we are looking to build them for ROS2: Foxy

## Docker
- Start Docker desktop (you should see the whale in the Windows tray)
- change to root directory

### ROS1 - Melodic
- To build docker image: `makemelodic.bat`
	- if successful you should be able to see a new image with `docker images`
- To run image: `startmelodic.bat`
    	- should see the `..../catkin_ws#` prompt

### ROS1 - Noetic
- To build docker image: `makenoetic.bat`
	- if successful you should be able to see a new image with `docker images`
- To run image: `startnoetic.bat`
      - should see the `..../catkin_ws#` prompt

### ROS2 - Foxy
- To build docker image: `makefoxy.bat`
	- if successful you should be able to see a new image with `docker images`
- To run image: `startfoxy.bat`
 

     

## Unity
### Background
- All of this is based on the "Unity Robotics Hub" tutorial project "Pick and Place", you should read that documentation to understand the baseline project
     - Home Page: https://github.com/Unity-Technologies/Unity-Robotics-Hub
     - Pick and Place tutorial: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md b
     - This Pick and Place tutorial was cut out of the original repo, the 3 submodules it had were "deleted" so that code is now just part of a single git repo
     - Some necessary files were in the `.gitignore` because they were considered "configurable" - those lines have been commmented out in the .gitignore because 
     - Khi robotics repo was added to the project but not as a submodule

### Instructions
- Open Unity Hub
- Create a project from a directory with `Open Project` 
    - choose any Unity 2020 LTS version
    - Browse to `KhiPickAndPlace`
    - Unless you get the same version the repo was last saved with, it should take awhile to open as it will need to convert the project
    - But then it should open without errors
- Once loaded there are various scenes you can load interesting ones are :
    - `Part3worksa` - A scene with the Niyro_one robot arm (the Robot arm thatis used in the Unity Robotics Hub demo)
    - `Part3worksb` - A scene with the Niyro_one robot arm but without a table, just a plane that everything rests on
    - `Part3worksc` - A scene with the rs007n KHI robot arm without a table, just a plane that everything rests on


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