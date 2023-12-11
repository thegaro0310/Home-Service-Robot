#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  move_base_msgs::MoveBaseGoal robot_pose;

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
	
  // set up the frame parameters
  robot_pose.target_pose.header.frame_id = "map";
  robot_pose.target_pose.header.stamp = ros::Time::now();
	
  // Define a position and orientation for the robot to reach
  robot_pose.target_pose.pose.position.x = 3.0;
  robot_pose.target_pose.pose.position.y = 1.0;
  robot_pose.target_pose.pose.position.z = 0.0;
  robot_pose.target_pose.pose.orientation.x = 0.0;
  robot_pose.target_pose.pose.orientation.y = 0.0;
  robot_pose.target_pose.pose.orientation.z = 0.0;
  robot_pose.target_pose.pose.orientation.w = 1.0;
	
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick-up position");
  ac.sendGoal(robot_pose);
	
  // Wait an infinite time for the results
  ac.waitForResult();
	
  // Check the robot reaches pick-up zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot has successed to move to the pick-up zone");
  }
  else
  {
    ROS_INFO("Robot has failed to move to the pick-up zone");
  }
	
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ros::Duration(5).sleep();
    robot_pose.target_pose.header.frame_id = "map";
    robot_pose.target_pose.header.stamp = ros::Time::now();

    // Define another position and orientation for the robot to reach
    robot_pose.target_pose.pose.position.x = -2.0;
    robot_pose.target_pose.pose.position.y = 3.0;
    robot_pose.target_pose.pose.position.z = 0.0;
    robot_pose.target_pose.pose.orientation.x = 0.0;
    robot_pose.target_pose.pose.orientation.y = 0.0;
    robot_pose.target_pose.pose.orientation.z = 0.0;
    robot_pose.target_pose.pose.orientation.w = 1.0;
		
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending drop-off position");
    ac.sendGoal(robot_pose);
		
    // Wait an infinite time for the results
    ac.waitForResult();
		
    // Check the robot reaches drop-off zone
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Robot has successed to move to the drop-off zone");
    }
    else
    {
      ROS_INFO("Robot has failed to move to the drop-off zone");
    }
  }
  return 0;
}
