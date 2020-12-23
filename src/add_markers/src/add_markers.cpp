#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

struct pose
{
  float x;
  float y;
  float theta;
};

pose pickupMarker = {-4.392, -1.259, 3.078};
pose dropoffMarker = {4.186, 0.114, 0.001};

enum RobotLocation_e
 {
    PICKUP_LOCATION = 0,
    DROPOFF_LOCATION = 1,
    ONROUTE_PICKUP = 2,
    ONROUTE_DROPOFF = 3,
    UNDEFINED = 4
};
uint8_t ROBOT_LOCATION = 4;

void goalReachCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  ROBOT_LOCATION = msg->data;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber goal_location_sub = n.subscribe("/goal_reach", 1, goalReachCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    // Listen to subcribed nodes
    ros::spinOnce();

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    switch(ROBOT_LOCATION)
    {
      case RobotLocation_e::ONROUTE_PICKUP:
      {
        // Set the pose of the marker at the pickup location.
        marker.pose.position.x = pickupMarker.x;
        marker.pose.position.y = pickupMarker.y;
        marker.pose.orientation.w = pickupMarker.theta;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        break;
      }

      case RobotLocation_e::PICKUP_LOCATION:
      {
        // Set the pose of the marker at the pickup location.
        marker.pose.position.x = pickupMarker.x;
        marker.pose.position.y = pickupMarker.y;
        marker.pose.orientation.w = pickupMarker.theta;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::DELETE;
        break;
      }
      
      case RobotLocation_e::ONROUTE_DROPOFF:
      {
        // Set the pose of the marker at the pickup location.
        marker.pose.position.x = pickupMarker.x;
        marker.pose.position.y = pickupMarker.y;
        marker.pose.orientation.w = pickupMarker.theta;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::DELETE;
        break;
      }
      
      case RobotLocation_e::DROPOFF_LOCATION:
      {
        // Set the pose of the marker at the pickup location.
        marker.pose.position.x = dropoffMarker.x;
        marker.pose.position.y = dropoffMarker.y;
        marker.pose.orientation.w = dropoffMarker.theta;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
        break;
      }
      
      default:
      {
        break;
      }
    }
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.001;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

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
    marker_pub.publish(marker);
    r.sleep();
  }
}