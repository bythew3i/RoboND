#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO("Drive bot lin_x = %.2f, ang_z = %.2f", lin_x, ang_z);

    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    float lin_x = 0;
    float ang_z = 0;
    float mid = (img.width - 1) / 2.0;
    for (int r = 0; r < img.height; r++)
    {
        for (int c = 0; c < img.width; c++)
        {
            // img.step = img.width * 3
            if (img.data[r * img.step + c * 3] == white_pixel &&
                img.data[r * img.step + c * 3 + 1] == white_pixel &&
                img.data[r * img.step + c * 3 + 2] == white_pixel)
            {
                int last = c + 1;
                while (last < img.width &&
                       img.data[r * img.step + last * 3] == white_pixel &&
                       img.data[r * img.step + last * 3 + 1] == white_pixel &&
                       img.data[r * img.step + last * 3 + 2] == white_pixel)
                    last++;
                float pos = (last - 1 + mid) / 2.0;
                if (pos > mid)
                {
                    lin_x = 0.5;
                    ang_z = -1.0;
                }
                else if (pos < mid)
                {
                    lin_x = 0.5;
                    ang_z = 1.0;
                }
                else
                {
                    lin_x = 0.5;
                    ang_z = 0.0;
                }
                break;
            }
        }
    }
    drive_robot(lin_x, ang_z);
}

int main(int argc, char **argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}