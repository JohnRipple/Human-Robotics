John Ripple
Project 2 Part 3
Human Centered Robotics
4/3/2023

How to run wall following robot
1) Start roscore
	roscore
2) Move ripple_wallFollowing into catkinws/src
3) Open a new terminal and remake files in ~/catkin_ws/
	catkin_make
4) Set up environment variables. Open ripple_wallFollowing_setup.bash and change CATKIN_WS=~/Human-Robotics/Project2_P3 to your workspace folder
5) Source the setup file in ~/catkin_ws/src/
	source ./ripple_wallFollowing_setup.bash
6) Source catkin ws environment in ~/catkin_ws
	source devel/setup.bash
7) run the launch file
	roslaunch ripple_wallFollowing ripple_wallFollowing.launch readQTable:=False train:=False
	
7a) readQTable tells the program to read in the pickle file Q table stored in ~/catkin_ws/src/ripple_wallFollowing/src/qTable.pkl
This file also saves the last QTable used by the program upon exit which will update every time the program is run and exited.
Default = False
Can be set to
	readQTable:=False
	readQTable:=True

7b) train tells the program to run the training which will start at a random position and do random actions for a certain number of episodes until a QTable is created.
Default = False
Can be set to
	train:=False
	train:=True

If you want to reset and run again without closing gazebo
1) rosservice call /gazebo/reset_simulation
2) rosrun ripple_wallFollowing ripple_wallFollowing.py
	train and readQTable will be set to false

Video Link to robot working, also included in the report
https://youtu.be/t5SGl2r8c8I
