/**
 * @file: process_image_node.h
 * @author: Ingrid Navarro
 * @date: February 20, 2020
 *
 * @brief: Drive bot node.
 */
#include "process_image/process_image.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "process_image");

	ProcessImage process_image;

	ros::spin();

	return 0;
}