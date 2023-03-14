John Ripple
Project 2 Part 2
Human Centered Robotics
3/8/2023

How to run wall following robot
1) Start roscore
	roscore
2) Move ripple_wallFollowing into catkinws/src
3) Open a new terminal
4) Set up environment variables. Open ripple_wallFollowing_setup.bash and change CATKIN_WS=~/Human-Robotics/Project2_P2 to your workspace folder
5) Run the setup file
	source ./ripple_wallFollowing_setup.bash
3) catkin_make
4) Open a new terminal in your catkin_ws and source environment
	source devel/setup.bash
5) run the launch file
	roslaunch ripple_wallFollowing wall_following_v1.launch

If you want to reset and run again without closing gazebo
1) rosservice call /gazebo/reset_simulation
2) rosrun ripple_wallFollowing ripple_wallFollowing.py

Video Link to robot working, also included in the report
https://youtu.be/1c6L9YDF6_M