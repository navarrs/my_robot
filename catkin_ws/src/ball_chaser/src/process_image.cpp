/**
 * @file: process_image.cpp
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Defines a class that subscribes to the robot's camera images and
 * analyzes them to determine the position of a white ball. 
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "ball_chaser/DriveToTarget.h"

class ProcessImage {
public:
	/**
	 * Constructor. 
	 * @param image_sub_topic[in]: Subscriber's topic name. 
	 * @param drive_bot_client_topic[in]: Client's topic name. 
	 * @param queue_size[in]: Message queue size.
	 */
	ProcessImage(
		const std::string &image_sub_topic = "/camera/rgb/image_raw", 
		const std::string &drive_bot_client_topic = "/ball_chaser/command_robot",
		const int &queue_size = 10)
	{
		// Client that requests services from /command_robot
		drive_bot_client_ = 
			process_image_node_.serviceClient<ball_chaser::DriveToTarget>(
				drive_bot_client_topic);

		// Subscriber to the image topic
		image_sub_ = process_image_node_.subscribe(image_sub_topic, queue_size, 
			&ProcessImage::ProcessImageCallback, this);

		ROS_INFO("ProcessImage ready");
	}

	/** 
	 * Drive robot in specified direction. 
	 * @param lin_x[in]: Translation along x. 
	 * @param ang_z[in]: Rotation about z. 
	 */
	void DriveRobot(const float &lin_x, const float &ang_z) 
	{
		ROS_INFO("Driving robot to target");

		// Request moving into specified direction
		ball_chaser::DriveToTarget srv;
		srv.request.linear_x = lin_x;
		srv.request.angular_z = ang_z;

		// Call the client to pass the direction 
		if(!drive_bot_client_.call(srv))
		{
			ROS_ERROR("Failed to call service to drive the robot");
		}
	}

	/** 
	 * Callback to process images. 
	 * @param img[in]: Received image.  
	 */
	void ProcessImageCallback(const sensor_msgs::Image img) 
	{
		const int white_pixel = 255;
		const int max_left = img.width / 3;
		const int max_forward = 2 * img.width / 3;

		for(int i = 0; i < img.height * img.step; ++i) {
			if(white_pixel == img.data[i]) {
				int col = (i % img.step) / 3;
				if(col >= 0 && col <= max_left) {
					// Move to the left
					DriveRobot(0.0, 0.5);
					return;
				} else if(col > max_left && col <= max_forward) {
					// Move forward
					DriveRobot(0.5, 0.0);
					return;
				} else {
					// Move right
					DriveRobot(0.0, -0.5);
					return;
				}
			}
		}
		// Stop
		DriveRobot(0.0, 0.0);
	}

private:

	// ROS Node 
	ros::NodeHandle process_image_node_;

	// Image subscriber 
	ros::Subscriber image_sub_;

	// Client to request services. 
	ros::ServiceClient drive_bot_client_;

	int queue_size_ = 10;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "process_image");

	ProcessImage process_image;

	ros::spin();

	return 0;
}