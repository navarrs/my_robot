/**
 * @file: process_image.cpp
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Defines a class that subscribes to the robot's camera images and
 * analyzes them to determine the position of a white ball. 
 */
#include "../include/process_image/process_image.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "process_image");

	ProcessImage process_image;

	ros::spin();

	return 0;
}