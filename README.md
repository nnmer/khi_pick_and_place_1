<p align="center"><img src="img/SimTrayToRail.gif"/></p>

# Kawaski RS007 Robot / Rockwell MagneMotion Pick-and-Place Demo

Derived from the Unity Robotics Hub Pick-and-Place tutorial
This unity app simulates the Rs007 Robot and Rockwell MagneMotion table features in the Build 2022 Keynote Demo (at the end of Satya Nadella's portion)

# Getting started
- prerequisites Windows 10 or 11, Docker Desktop, Unity
- You will want to build the ROS Docker images 
   - we have scripts to build and run them for ROS1: Melodic and Noetic
   - we are looking to build them for ROS2: Foxy but they are not done yet

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
      - you can then start a launch file for eample with `. devel/rs007_launch.txt`
      
- To open a sencond window enter:
      - `newroswindow.bat`
      - `. devel\setup.bash`
      - `rosnode list`
      
# Notes
The application has different modes:
  - Simulation mode in which it can run in 
       - moving boxes from rail to rail
       - moving boxes from tray to rail
       - moving boxes from rail to tray
  - Echo mode
       - echos the box and robot motions that are occuring somewhere else, either on a physical robot or on a virtual one (for example another instance of this appoication)


# Compiling the Unity Project
- The Unity Project root is in `KhiPickAndPlaceProject` Subdirectory in this repo, that is where you have to point Unity Hub
- The packages needed should install themselves, I beleive - not sure about this
- One small change needs to be made after the UnityRoboticsHub package is installed to make this compile:
   - the `KhiPickAndPlaceProject\Library\PackageCache\com.unity.robotics.ros-tcp-connector@c27f00c6cf\Runtime\TcpConnector\ROSConnection.cs` file has a method (`InitializeHUD`) that needs to be made public like this on line 1016:
   - `public void InitializeHUD()` 
   - I will see if I can find another workaround for it later

   

Keyboard Commands:

   Ctrl-E Echo Mode
   Ctrl-P RailToRail Mode
   Ctrl-L RailToRail Mode
   Ctrl-T TrayToRail Mode
   Ctrl-R Reverse TrayRail
            
   Ctrl-F Speed up
   Ctrl-S Slow down
            
   Ctrl-N Toggle Enclosure
   Ctrl-D Toggle Stop Simulation
   Ctrl-G Toggle Log Screen
            
   Ctrl-V Ctrl-F View from Front
   Ctrl-V Ctrl-B View from Back
   Ctrl-V Ctrl-T View from Top
   Ctrl-V Ctrl-S View from Top (rotated)
   Ctrl-V Ctrl-R View from Right
   Ctrl-V Ctrl-L View from Left
            
   Ctrl-H Toggle Help Screen
   Ctrl-Q Ctrl-Q Quit Application

Parameters:
   --roshost localhost
   --zmqhost localhost
   --rosport 10004
   --zmqport 10006
   --mode echo
   --mode rail2rail
   --mode rail2rail
   --mode tray2rail
   --mode rail2tray

# Addresses as of 15 July 2022:
   Western Europe - 20.234.234.190
   USA -  20.225.161.122

 
 
 # Start two instances with one echoing the other
   - Make sure you have a ROS container running and know what port it is using for the ROS-Unity communication (below example localhost:10005)
   
   - Open a cmd window
   - Enter: `khidemosim --mode tray2rail --zmqhost localhost --zmqport 10006`
   - (note: If you are using a local host you should see output echoing the statusrunning in the container window)
   - Open a second cmd window
   - Enter: `khidemosim --mode echo --roshost localhost --rosport 10005`
   - The upper left communication HUD IP window should be showing green communication arrows

