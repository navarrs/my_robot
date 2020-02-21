/**
 * @file: process_image.cpp
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Implements a class that subscribes to the robot's camera images and
 * analyzes them to determine the position of a white ball. 
 */
#include "process_image.h"

ProcessImage::ProcessImage(const std::string &image_sub_topic, 
	const std::string &drive_bot_client_topic, const int &queue_size) {
	// Client that requests services from /command_robot
	drive_bot_client_ = 
		process_image_node_.serviceClient<ball_chaser::DriveToTarget>(
			drive_bot_client_topic);

	// Subscriber to the image topic
	image_sub_ = process_image_node_.subscribe(image_sub_topic, queue_size, 
		&ProcessImage::ProcessImageCallback, this);

	ROS_INFO("ProcessImage ready");
}

void ProcessImage::DriveRobot(const float &lin_x, const float &ang_z) {
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

void ProcessImage::ProcessImageCallback(const sensor_msgs::Image img) {
	const int white_pixel = 255;
	const int max_left = img.width / 3;
	const int max_forward = 2 * img.width / 3;

	for(int i = 0; i < img.height * img.step; ++i) {
		if(white_pixel == img.data[i]   &&  // Check if pixel is (255, 255, 255)
			 white_pixel == img.data[++i] &&
			 white_pixel == img.data[++i] ) {
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