# RobotModellingAndSimulation

Instructions to run the package:
1) Build the package before running (Since, the build & devel folders included have absolute paths)
2) Place the given package in the catkin workspace folder. 
3) Commands to run: 
    1) cd ~/catkin_ws
        1) Change directory to the catkin workspace on your local machine. 
    2) catkin_make robot
        1) Build the package. 
    3) . devel/setup.bash
        1) Source the bash file of the package
    4) roslaunch robot template_launch.launch
        1) Setup gazebo world & spawn the robot.

4) Scripts Description (src/robot/scripts):
    1) arm_control.py
        1) Script to move the arm.
    2) teleop.py
        1) Script to move the mobile robot
    3) forward_kinematics_transform.py
        1) Script to validate forward kinematics
    4) inverse_kinematics.py
        1) Script to validate inverse kinematics
