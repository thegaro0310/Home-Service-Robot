#!/bin/sh

# <><><><><>
# TURTLEBOT
# <><><><><>
# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x 5 -y 1 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/my_house.world " & 

sleep 5

# <><><><><><>
# --- AMCL ---
# <><><><><><>
# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../../src/map/map.yaml " &

sleep 15

# <><><><><><>
# --- RVIZ ---
# <><><><><><>
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch file_config:=$(pwd)/../rvizConfig/home_service.rviz" &

sleep 15 # keeping large to enable visualization

# <><><><><><>
# ADD MARKERS
# <><><><><><>
# launch add_markers node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun add_markers add_markers_test" &
