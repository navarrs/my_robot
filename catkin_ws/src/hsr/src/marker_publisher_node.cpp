/** ----------------------------------------------------------------------------
 * @file: marker_publisher_node.cpp
 * @date: August 14, 2020
 * @author: navarrs
 * 
 * @brief: Launches marker_publisher node based on the MarkerPublisher class. 
 * ---------------------------------------------------------------------------*/

// INCLUDES --------------------------------------------------------------------
#include "marker_publisher/marker_publisher.h"

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
using namespace hsr;

// ENUMS -----------------------------------------------------------------------

// STRUCTS ---------------------------------------------------------------------

// GLOBAL PARAMETERS -----------------------------------------------------------

// FUNCTION DECLARATION --------------------------------------------------------

// MAIN PROGRAM ----------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle node("marker_publisher");
	service_robot::MarkerPublisher mp(node);
	if(Exit::SUCCESS != mp.Initialize()) 
  {
		printf("Could not initialize marker_publisher node\n");
		return EXIT_FAILURE;
	}
  ros::spin();
	return 0;
}

// FUNCTION IMPLEMENTATION -----------------------------------------------------