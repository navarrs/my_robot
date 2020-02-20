/**
 * @file: drive_bot.h
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Defines a class that will provide a service to drive the robot 
 * by controlling its linear x and angular z velocities. 
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "ball_chaser/DriveToTarget.h"

class DriveBot {
public:
	/**
	 * Constructor. 
	 * @param motor_command_pub_topic[in]: Publisher's topic name. 
	 * @param drive_bot_srv_topic[in]: Service's topic name. 
	 * @param queue_size[in]: Message queue size.
	 */
	DriveBot(
		const std::string &motor_command_pub_topic = "/cmd_vel", 
		const std::string &drive_bot_srv_topic = "/ball_chaser/command_robot",
		const int &queue_size = 10);

	/** 
	 * Callback to handle drive requests. 
	 * @param req[in]: DriveToTarget message request. 
	 * @param res[in]: DriveToTarget message response. 
	 */
	bool HandleDriveRequest(
		ball_chaser::DriveToTarget::Request &req, 
		ball_chaser::DriveToTarget::Response &res);

private:

	// ROS Node 
	ros::NodeHandle drive_bot_node_;

	// Publisher for the motor commands.
	ros::Publisher motor_command_pub_;

	// Service to handle motor command requests. 
	ros::ServiceServer drive_bot_srv_;
};