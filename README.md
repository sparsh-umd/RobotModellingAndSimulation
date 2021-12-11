# RobotModellingAndSimulation

Instructions to run the package:
1) Build the package before running (Since, the build & devel folders included have absolute paths)
2) Place the given package in the catkin workspace folder. 
3) Commands to run: 
  a) cd ~/catkin_ws
      Change directory to the catkin workspace on your local machine. 
  b) catkin_make robot
      Build the package. 
  c) . devel/setup.bash
      Source the bash file of the package
  d) roslaunch robot template_launch.launch
      Setup gazebo world & spawn the robot.
