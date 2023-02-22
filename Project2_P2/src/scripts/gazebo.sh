export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/Human-Robotics/Project2_P2/src/saved-data/map.yaml
