/**
 * @file: drive_bot.cpp
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Implements a class that will provide a service to drive the robot 
 * by controlling its linear x and angular z velocities. 
 */
#include "drive_bot.h"

DriveBot::DriveBot(
	const std::string &motor_command_pub_topic, 
	const std::string &drive_bot_srv_topic,
	const int &queue_size)
{
	motor_command_pub_ = drive_bot_node_.advertise<geometry_msgs::Twist>(
		motor_command_pub_topic, queue_size);

	drive_bot_srv_ = drive_bot_node_.advertiseService(
		drive_bot_srv_topic, &DriveBot::HandleDriveRequest, this);

	ROS_INFO("DriveBot ready");
}

bool DriveBot::HandleDriveRequest(
	ball_chaser::DriveToTarget::Request &req, 
	ball_chaser::DriveToTarget::Response &res) 
{
	ROS_INFO("Drive to target received - lin-x:%1.2f, ang_z:%1.2f", 
		(float)req.linear_x, (float)req.angular_z);

	// Create a motor_command object of type geometry_msgs::Twist
	geometry_msgs::Twist motor_command;
	
	// Set wheel velocities
	motor_command.linear.x = req.linear_x;
	motor_command.angular.z = req.angular_z;

	// Publish motor command
	motor_command_pub_.publish(motor_command);

	// Return a response message
	res.msg_feedback = "Motor command - lin_x: " + std::to_string(req.linear_x) + 
										  ", ang_z: " + std::to_string(req.angular_z);
	ROS_INFO_STREAM(res.msg_feedback);

	return true;
}