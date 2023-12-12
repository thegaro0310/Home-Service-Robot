# Home Service Robot Project Overview

This repository serves as the submission for the "Home Service Robot" project in the UDACITY Robotics Software Engineer program, specifically within the Home Service Robot course. The project requires the application of acquired knowledge to deploy a home service robot tasked with picking up an object and delivering it to a different location.

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
