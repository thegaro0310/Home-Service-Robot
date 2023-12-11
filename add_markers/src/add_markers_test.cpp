#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

typedef enum
{
  pickup = 0,
  dropoff = 1,
}marker_state;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_state state;
  state = pickup;

  // Subscribe to odometry values
  // ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, odometry_cb);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

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

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.lifetime = ros::Duration();

    switch(state)
    {
      case pickup:
      {
        // <><><><><><><><><><><>
        // ---- PICK UP ZONE ----
        // <><><><><><><><><><><>

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        // Spawn object at pick-up zone
        marker.action = visualization_msgs::Marker::ADD;
        ROS_INFO("Spawn marker object at pick-up zone");
        marker.pose.position.x = 3.0;
        marker.pose.position.y = 1.0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker_pub.publish(marker);
  
        // Wait for 5 seconds
        sleep(5);

        state = dropoff;
        break;
      }
      case dropoff:
      {
        // <><><><><><><><><><><>
        // ---- DROPOFF ZONE ----
        // <><><><><><><><><><><>

        // Hide object after 5 seconds
        marker.action = visualization_msgs::Marker::DELETE;
        ROS_INFO("Hide marker object at pick-up zone");

        // Wait for another 5 seconds
        sleep(5);

        // Spawn object at drop-off zone
        marker.action = visualization_msgs::Marker::ADD;
        ROS_INFO("Spawn marker object at pick-up zone");
        marker.pose.position.x = -2.0;
        marker.pose.position.y = 3.0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker_pub.publish(marker);

        break;
      }
    }
  }
  return 0;
}
