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
		const int &queue_size = 10)
	{
		motor_command_pub_ = drive_bot_node_.advertise<geometry_msgs::Twist>(
		motor_command_pub_topic, queue_size);

		drive_bot_srv_ = drive_bot_node_.advertiseService(
			drive_bot_srv_topic, &DriveBot::HandleDriveRequest, this);

		ROS_INFO("DriveBot ready");
	}

	/** 
	 * Callback to handle drive requests. 
	 * @param req[in]: DriveToTarget message request. 
	 * @param res[in]: DriveToTarget message response. 
	 */
	bool HandleDriveRequest(
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

private:

	// ROS Node 
	ros::NodeHandle drive_bot_node_;

	// Publisher for the motor commands.
	ros::Publisher motor_command_pub_;

	// Service to handle motor command requests. 
	ros::ServiceServer drive_bot_srv_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_bot");

	DriveBot drive_bot;

	ros::spin();

	return 0;
}