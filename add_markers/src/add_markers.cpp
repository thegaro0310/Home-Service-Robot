#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Define object states
bool check_pick_up = 0;
bool check_drop_off = 0;

// Define pick_up and drop_off goals
float pose_pickup_x = 3.0;
float pose_pickup_y = 1.0;
float pose_dropoff_x = -2.0;
float pose_dropoff_y = 3.0;
float goal_reach = 0.3;

// Callback function for odometry
void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
  float distance_pickup;
  float distance_dropoff;
  float pose_x = msg->pose.pose.position.x;
  float pose_y = msg->pose.pose.position.y;
	
  if(check_pick_up == 0 && check_drop_off == 0)
  {
    distance_pickup = sqrt(pow((pose_pickup_y - pose_y), 2) + 
			   pow((pose_pickup_x - pose_x), 2));
    ROS_INFO("It's %f to the pick-up zone", distance_pickup);
    if(goal_reach > distance_pickup)
    {
      ROS_INFO("Robot has arrived at the pick-up zone");
      check_pick_up = 1;
    }
  }
  else if(check_pick_up == 1)
  {
    distance_dropoff = sqrt(pow((pose_dropoff_y - pose_y), 2) + 
			    pow((pose_dropoff_x - pose_x), 2));
    ROS_INFO("It's %f to the drop-off zone", distance_dropoff);
    if(goal_reach > distance_dropoff)
    {    
      ROS_INFO("Robot has arrived at the drop-off zone");
      check_pick_up = 0;
      check_drop_off = 1;
    }
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  int display_counter_pick_up = 0;
  int display_counter_drop_off = 0;

  // Subscribe to odometry values
  ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, odometry_cb);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Initialize marker
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
	
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pose_pickup_x;
  marker.pose.position.y = pose_pickup_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
	
  marker.lifetime = ros::Duration();

  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    // Check pick-up condition
    if(check_pick_up == 1)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      display_counter_pick_up++;
      if(display_counter_pick_up == 1)
      {
        ROS_INFO("Robot picks object up");
      }
      ros::Duration(5.0).sleep();
    }
    // Check drop-off condition
    if(check_drop_off == 1)
    {
      marker.pose.position.x = pose_dropoff_x;
      marker.pose.position.y = pose_dropoff_y;
      marker.action = visualization_msgs::Marker::ADD;
      display_counter_drop_off++;
      if(display_counter_drop_off == 1)
      {
        ROS_INFO("Robot drops object off");
      }
      ros::Duration(5.0).sleep();
    }

    marker_pub.publish(marker);
    ros::spinOnce();
  }
  return 0;
}
