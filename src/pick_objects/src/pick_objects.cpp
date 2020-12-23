#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

int main(int argc, char** argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_Objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
   
   // Define the pick-up position and orientation for the robot to reach
    ROS_INFO("Sending pick-up location");    
    setNewGoal(ac, -4.429, -1.325, -0.036); 

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the robot picked up the object");
    else
        ROS_INFO("The robot failed to pick up object for some reason");

    // Wait for 5 seconds before proceeding
    ros::Duration(5.0).sleep();

    // Define the drop-off position and orientation for the robot to reach
    ROS_INFO("Sending drop-off location");    
    setNewGoal(ac, 4.203, 0.015, 3.119); 

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the robot picked up the object");
    else
        ROS_INFO("The robot failed to pick up object for some reason");

    // Wait for 15 seconds before exiting
    ros::Duration(15.0).sleep();

    return 0;
}

