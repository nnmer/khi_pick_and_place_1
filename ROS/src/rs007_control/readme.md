# RVIZ robotic control with python

- copy entire rs007_control directory
   - In WSL
   - change to ~/ros/catkin_ws/src
   - ./get_rs007_control
   - It will copy and echo the file sit copies
   

- Need two windows
  - Only works in WSL - at least for noetric
  - `roslaunch rs007_control rvizdemo.launch`
  - `cd rs007_control/scripts`
  - `python rs007n_demo.py`
  - Interrupt with Ctrl-z, or Ctrl-d (sometimes)
  
 - Show markers
   - Add MarkerArray visualizer to RVIZ window with `Add` button
  - `cd rs007_control/scripts`
  - `python marker1.py`
