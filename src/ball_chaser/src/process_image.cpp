#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// Call the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    //ROS_INFO_STREAM("Driving the robot towards the ball");

    // Request service
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service
    if(!client.call(srv))
      ROS_ERROR("Failed to call service command_robot");
}


// Callback function that continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // Book keeping variables
    int white_pixel = 255;
    int num_white_pixel = 1;
    int x_position_sum = 0;

    // Create an enum for all possible ball locations
    enum Location_e 
    { 
      LEFT = 0,
      MIDDLE,
      RIGHT,
      NONE
    };
    Location_e ball_position = Location_e::NONE;

    // Loop through all the pixels in the image and check for white pixels
    for(int i = 0; i < img.data.size(); i+=3)
    {
      auto red = img.data[i];
      auto green = img.data[i+1];
      auto blue = img.data[i+2];
      if(red == white_pixel && green == white_pixel && blue == white_pixel)
      {
	      ++num_white_pixel;
	      int x_position = (i % (img.width * 3)) / 3;
        x_position_sum += x_position;
      } 
    }

    // Compute the centroid of the ball and determine its location in the image
    int x_position_mean = x_position_sum/num_white_pixel;
    if(x_position_mean> 0 && x_position_mean < img.width / 3)
    {
      ball_position = Location_e::LEFT;
      ROS_INFO("Left turn detected");
    }
    else if(x_position_mean > img.width * 2/3)
    {
      ball_position = Location_e::RIGHT; 
      ROS_INFO("Right turn detected");
    }
    else
    {
      ball_position = Location_e::MIDDLE;
    }

    // Stop the robot if the ball doesn't exist in its FOV or it's too close
    if(ball_position == Location_e::NONE || num_white_pixel < 10 || num_white_pixel > 50000)
    {
       drive_robot(0.0, 0.0);
    }
    // Drive the robot towards the ball
    else if(ball_position == Location_e::LEFT)
    {
      drive_robot(0.0, 0.5);      
    }

    else if(ball_position == Location_e::RIGHT)
    {
      drive_robot(0.0, -0.5);      
    }

    else if(ball_position == Location_e::MIDDLE)
    {
      drive_robot(0.5, 0.0);
    }
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_cb function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    ROS_INFO("Receiving image data");

    return 0;
}
