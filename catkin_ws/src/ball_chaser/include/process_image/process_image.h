/**
 * @file: process_image.h
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
		const int &queue_size = 10);

	/** 
	 * Drive robot in specified direction. 
	 * @param lin_x[in]: Translation along x. 
	 * @param ang_z[in]: Rotation about z. 
	 */
	void DriveRobot(const float &lin_x, const float &ang_z);

	/** 
	 * Callback to process images. 
	 * @param img[in]: Received image.  
	 */
	void ProcessImageCallback(const sensor_msgs::Image img);

private:

	// ROS Node 
	ros::NodeHandle process_image_node_;

	// Image subscriber 
	ros::Subscriber image_sub_;

	// Client to request services. 
	ros::ServiceClient drive_bot_client_;

	int queue_size_ = 10;
};