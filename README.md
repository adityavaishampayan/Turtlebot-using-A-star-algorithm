# Turtlebot-using-A-algorithm

The following files have been included in the submission folder.
1)  src folder
2) final_submit.py file (Python file, also present in src/turtlebot_astar/scripts/)
3) Video of simulation
4) Readme file


Instructions for creating ROS catkin_workspace:
Run these commands in the terminal to create ROS workspace
1.  source /opt/ros/kinetic/setup.bash
2.  mkdir -p catkin_ws/src
3.cd catkin_ws/

unzip the submitted file and paste the src in the catkin_ws folder

The python file is in the src/script folder. To make the script an executable file, perform step 4 :
4. chmod +x final_submit.py
5. catkin_make
6. source devel/setup.bash
7. export TURTLEBOT3_MODEL=waffle
8. roslaunch astar_differential astar.launch x_pos:=0.0 y_pos:=0.0 (Here x and y are in gazebo coordinate frame) I.e. if start coordinates  in generated map is at (955, 505) , we spawn the robot at x_pos:= 4.0 and y_pos:=0.0
