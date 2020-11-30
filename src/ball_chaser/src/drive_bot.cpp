#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Global joint publisher variables
ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, 
				ball_chaser::DriveToTarget::Response& res)
{
    // Set linear and angular velocities to a local variable
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    ROS_INFO("DriveToTarget request received with linear vel: %1.2f, angular vel: %1.2f", 
	(float)req.linear_x, (float)req.angular_z);

     // Publish angles to drive the robot
     motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Motor commands set to- linear: " + std::to_string(motor_command.linear.x) 
	+ ", angular: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Define a publisher to publish a message of type geometry_msgs::Twist on the robot actuation
    // topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send velocity commands");

    // Handle ROS communication events
    ros::spin();

    return 0;

}

