# Home Service Robot Project Overview
This repository serves as the submission for the "Home Service Robot" project in the UDACITY Robotics Software Engineer program, specifically within the Home Service Robot course. The project requires the application of acquired knowledge to deploy a home service robot tasked with picking up an object and delivering it to a different location.

## Brief Write-up
### 1. Localization:
#### Importance of Localization:
Localization is a fundamental aspect that enables a robot to precisely determine its position and orientation within its environment, facilitating effective navigation and interaction with its surroundings. 
#### Localization and Mapping Algorithms: AMCL (Adaptive Monte Carlo Localization) and GMapping:
For the localization component, two key algorithms are employed — Adaptive Monte Carlo Localization (AMCL) and GMapping. AMCL uses a particle filter approach for adaptive pose estimation, while GMapping is a grid-based mapping algorithm that leverages laser range data to create a 2D occupancy grid map.
#### Move Base: A Comprehensive Navigation Package:
In conjunction with AMCL and GMapping, Move Base is utilized as a comprehensive navigation package. Move Base combines the outputs of AMCL and GMapping to plan and execute robot motion towards a specified goal location. This integrated navigation system ensures a cohesive and efficient approach to guiding the robot through its environment.
#### How Move Base Works:
Move Base integrates the localization information provided by AMCL and the environmental mapping from GMapping. It then employs path planning algorithms to determine a feasible trajectory for the robot to reach its goal. The system continuously adjusts the path based on real-time sensor feedback, allowing the robot to navigate dynamically changing environments.
#### Challenges and Insights in Implementation:
Implementing Move Base involves addressing challenges related to coordinating information from AMCL and GMapping, optimizing path planning, and ensuring seamless execution of robot motion. Tuning parameters for Move Base becomes crucial to strike a balance between precision and efficiency in navigation.
#### Lessons Learned:
The inclusion of Move Base in the navigation system highlighted the importance of a holistic approach to localization and path planning. The integration of AMCL and GMapping, coupled with Move Base, emphasized the need for careful parameter tuning and continual refinement to ensure the reliable and efficient navigation of the home service robot across diverse environmental scenarios.

### 2. Mapping:
#### Significance of Mapping:
Mapping is a critical component in facilitating a robot's understanding and navigation within its environment. The primary objective is to enable the robot to construct a detailed representation of its surroundings, empowering it to make intelligent decisions, navigate around obstacles, and plan optimal routes.
#### Mapping Algorithm: SLAM (Simultaneous Localization and Mapping):
For the mapping component of this project, the chosen algorithm is Simultaneous Localization and Mapping (SLAM). SLAM is a sophisticated technique that empowers a robot to simultaneously localize itself within an environment while creating an up-to-date map, in our project it's our world which we created in previous leasons. This capability is essential for a home service robot to function autonomously and navigate through dynamically changing spaces.
#### SLAM Functionality:
SLAM operates by assimilating sensor data, typically from cameras, lidars, or other perception sensors, to construct a comprehensive map of the environment. Simultaneously, the algorithm continually estimates the robot's position within this map. This dual process of localization and mapping is crucial for providing the robot with an accurate understanding of its surroundings as it moves.
#### Parameterization and Configurations:
In the implementation of SLAM, specific parameters and configurations played a pivotal role in achieving a high-quality map. Parameters such as sensor noise, mapping resolution, and loop closure thresholds underwent meticulous tuning to optimize the overall performance of the SLAM algorithm. These configurations directly influenced the accuracy of the generated map and the robot's capacity to localize itself effectively.
#### Challenges in the Mapping Process:
The mapping process encountered its share of challenges. Adapting SLAM to diverse environments, addressing sensor noise, and fine-tuning the algorithm for real-world scenarios presented ongoing obstacles. Continuous refinement of parameters and vigilant monitoring of the mapping process were imperative to tackle issues such as drift and loop closures.
#### Lessons Learned:
This mapping experience underscored the importance of thorough parameter tuning, a deep understanding of sensor limitations, and the necessity to adapt SLAM to varying environmental conditions. It emphasized the need for a mapping solution that is robust and adaptive, ensuring the reliable and efficient navigation of the home service robot in diverse settings.

### 3. Navigation:
#### Importance of Navigation
Navigation is a crucial aspect of robotics that enables a robot to plan and execute paths to reach different locations in its environment. Without navigation, a robot would not be able to autonomously move and interact with its surroundings effectively.
#### Navigation Algorithm
In the context of the ROS (Robot Operating System) ecosystem, the ROS Navigation Stack is a widely-used package for navigation. It provides a set of algorithms and tools that allow a robot to navigate autonomously in a known environment. The Navigation Stack utilizes the map generated by SLAM (Simultaneous Localization and Mapping) to plan paths and avoid obstacles.
#### Navigation Stack Functionality:
The Navigation Stack consists of several components, including a map server, a localization system (such as AMCL), a global planner, and a local planner. The map server provides the occupancy grid map generated by SLAM, which represents the environment with obstacles and free space.
- The global planner takes the map as input and plans a high-level path from the robot's current pose to the goal pose. It uses algorithms like Dijkstra or A* to find the optimal path while considering obstacles. The global planner generates a sequence of waypoints that the robot should follow to reach the goal
- The local planner, on the other hand, is responsible for generating a smooth and feasible trajectory for the robot to follow. It takes into account the robot's dynamics, sensor data, and local obstacles to generate a path that avoids collisions. The local planner continuously adjusts the robot's trajectory based on the current sensor data and ensures that the robot can navigate safely in real-time.
#### Navigation Stack parameters configuration:
- Configuring the parameters in the Navigation Stack is crucial for achieving effective path planning and execution. Some important parameters include the inflation radius, which determines how much space the robot should keep away from obstacles, and the planner frequency, which determines how often the planner should generate new paths.
- Tuning these parameters requires a balance between safety and efficiency. A larger inflation radius can help the robot avoid collisions but may result in a less optimal path. On the other hand, a smaller inflation radius may lead to potential collisions. Similarly, adjusting the planner frequency affects the responsiveness of the robot's navigation but can also impact computational resources.
#### Challenges in the Navigation Process:
Implementing navigation can come with its own set of challenges and insights. Some challenges include tuning the parameters to achieve smooth and collision-free navigation, handling dynamic obstacles, and ensuring accurate localization. It's important to thoroughly test and iterate on the navigation system to address these challenges and improve the robot's performance.

--------------------------------------------------------------------------------

## Project Components
### SLAM Mapping
To initiate SLAM testing, a `test_slam.sh` script is provided. This script launches the necessary components for manual SLAM testing. The objective is to generate a functional map of the environment, crucial for subsequent localization and navigation tasks.
### Localization and Navigation
For manual navigation testing, the `test_navigation.sh` script is included. This script facilitates the manual navigation of the robot, allowing it to respond to 2D Nav Goal commands. The `pick_objects.sh` script sends multiple goals for the robot to achieve. The robot navigates to the designated pickup zone, confirms its arrival, waits for 5 seconds, proceeds to the assigned drop-off zone, and confirms its arrival.
### Home Service Functions
To incorporate home service functionality, the `add_marker.sh` script is introduced. This script publishes a marker to rviz, initially positioned at the pickup zone. After 5 seconds, the marker disappears and reappears at the drop-off zone after another 5 seconds. The `home_service.sh` script orchestrates the entire project by running all the necessary nodes. The simulated home service robot follows these steps:

1. Display the marker initially at the pickup zone.
2. Hide the marker upon the robot's arrival at the pickup zone.
3. Simulate a 5-second wait for the pickup.
4. Display the marker at the drop-off zone once the robot reaches it.

## Directory structure
```
├── add_markers                                   # add_markers package        
│   ├── src
│   │   ├── add_markers.cpp                       
│   │   ├── add_markers_test.cpp                  # for testing add_markers node purpose
├── map                                           # map and world file       
├── pick_objects                                  # pick_objects package     
│   ├── src                                       
│   │   ├── pick_objects.cpp                      
├── rvizConfig                                    # rvizConfig file for home service robot demo     
│   ├── home_service.rviz                         
├── scripts                                       # shell scripts folder
│   ├── add_marker.sh                             
│   ├── home_service.sh                           
│   ├── pick_objects.sh                           
│   ├── test_navigation.sh                        
│   ├── test_slam.sh                              
├── slam_gmapping                                 
├── turtlebot                                    
├── turtlebot_interactions                      
├── turtlebot_simulator                           
├── CMakeLists.txt                                # make file
```
- [add_markers.cpp](/catkin_ws/src/add_markers/src/add_markers.cpp): This C++ script facilitates communication with the `pick_objects` node and manages marker visibility to simulate object pick-up and drop-off events.
- [pick_objects.cpp](/catkin_ws/src/pick_objects/src/pick_objects.cpp): A C++ script designed to interact with the `add_markers` node and command the robot to pick up objects.
- [home_service.rviz](/catkin_ws/src/rvizConfig/home_service.rviz): The `home_service.rviz` file, part of the rvizConfig, configures the visualization for the home service robot demo, including options related to the display of markers.
- [test_navigation.sh](/catkin_ws/src/scripts/test_navigation.sh): A shell script file that deploys a TurtleBot in the environment, sets two distinct goals, and evaluates the robot's ability to reach and orient itself with respect to these goals.
- [test_slam.sh](/catkin_ws/src/scripts/test_slam.sh): A shell script file responsible for deploying a TurtleBot, controlling it through keyboard commands, interfacing with a SLAM package, and visualizing the generated map in `rviz`.
- [add_marker.sh](/catkin_ws/src/scripts/add_marker.sh): A shell script file that deploys a TurtleBot, models a virtual object with markers in `rviz`, and demonstrates marker-related functionalities.
- [pick_objects.sh](/catkin_ws/src/scripts/pick_objects.sh): A shell script file that deploys a TurtleBot, communicates with the ROS navigation stack, and autonomously sends successive goals for the robot to achieve.
- [home_service.sh](/catkin_ws/src/scripts/home_service.sh): A shell script file designed to deploy a TurtleBot, simulating a comprehensive home service robot capable of navigating to pick up and deliver virtual objects.
