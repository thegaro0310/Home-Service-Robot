#!/bin/sh

# turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..; source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x 5 -y 1 -z 0 -R 0 -P 0 -Y -4.71';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/my_house.world " &

sleep 10

# amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..; source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../../src/map/map.yaml " &

sleep 10

# rviz for visualization
xterm -e "cd $(pwd)/../..; source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch file_config:=$(pwd)/../rvizConfig/home_service.rviz" &

sleep 10

# add_markers
xterm -e "cd $(pwd)/../..; source devel/setup.bash;
rosrun add_markers add_markers " &

sleep 10

# pick_objects
xterm -e "cd $(pwd)/../..; source devel/setup.bash;
rosrun pick_objects pick_objects" &
