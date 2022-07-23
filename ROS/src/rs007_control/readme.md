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


- Running Tutorials
  - Moveit C++ Interface
    - Link: (https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html#planning-with-path-constraints)
    - Ubuntu shell1: `roslaunch panda_moveit_config demo.launch`
    - Ubuntu shell2: `roslaunch moveit_tutorials move_group_interface_tutorial.launch`