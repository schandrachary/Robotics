#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

struct pose
{
  float x;
  float y;
  float theta;
};

pose pickup = {-4.072, -1.283, 3.111};
pose dropoff = {3.840, 0.082, 0.058};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Declare constant
std_msgs::UInt8 ROBOT_LOCATION;

bool setNewGoal(MoveBaseClient& ac, const float& x, const float& y, const float& angle)
{
    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Send the goal position and orientation for the robot to reach
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = angle;

    // Send goal
    ac.sendGoal(goal);

    return true;
}

enum RobotLocation
 {
    PICKUP_LOCATION = 0,
    DROPOFF_LOCATION = 1,
    ONROUTE_PICKUP = 2,
    ONROUTE_DROPOFF = 3,
    UNDEFINED = 4
};

int main(int argc, char** argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_Objects");
    ros::NodeHandle n;
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Set up publisher to broadcast if robot is at the gaol position
    ros::Publisher robot_location_pub = n.advertise<std_msgs::UInt8>("/goal_reach", 1);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
   
   // Define the pick-up position and orientation for the robot to reach
    ROS_INFO("Sending pick-up location");    
    setNewGoal(ac, pickup.x, pickup.y, pickup.theta); 

    // Publish pickup onroute message ONROUTE_PICKUP
    ROBOT_LOCATION.data = 2; 
    robot_location_pub.publish(ROBOT_LOCATION);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the robot picked up the object");
        ROBOT_LOCATION.data = 0;
        robot_location_pub.publish(ROBOT_LOCATION);
    }
    else
        ROS_INFO("The robot failed to pick up object for some reason");

    // Wait for 5 seconds before proceeding
    ros::Duration(5.0).sleep();

    // Define the drop-off position and orientation for the robot to reach
    ROS_INFO("Sending drop-off location");    
    setNewGoal(ac, dropoff.x, dropoff.y, dropoff.theta); 

    // Publish dropoff onroute message ONROUTE_DROPOFF
    ROBOT_LOCATION.data = 3;
    robot_location_pub.publish(ROBOT_LOCATION);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {   
        ROS_INFO("Hooray, the robot droped off the object");
        // Publish dropoff onroute message DROPOFF_LOCATION
        ROBOT_LOCATION.data = 1;
        robot_location_pub.publish(ROBOT_LOCATION);
    }
    else
        ROS_INFO("The robot failed to pick up object for some reason");

    // Wait for 15 seconds before exiting
    ros::Duration(15.0).sleep();

    return 0;
}

