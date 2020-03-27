/**
 * @file: drive_bot.h
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Defines a class that will provide a service to drive the robot 
 * by controlling its linear x and angular z velocities. 
 */
#include "../include/drive_bot/drive_bot.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_bot");

	DriveBot drive_bot;

	ros::spin();

	return 0;
}