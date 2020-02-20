/**
 * @file: drive_bot_node.h
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Drive bot node.
 */
#include "drive_bot/drive_bot.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_bot");

	DriveBot drive_bot;

	ros::spin();

	return 0;
}