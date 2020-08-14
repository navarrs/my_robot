/** ----------------------------------------------------------------------------
 * @file: marker_command_req.cpp
 * @date: August 14, 2020
 * @author: navarrs
 * 
 * @brief: Requests a marker action 
 * ---------------------------------------------------------------------------*/

// INCLUDES --------------------------------------------------------------------
#include "marker_publisher/marker_publisher.h"
#include "hsr/MarkerCommand.h"

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------

// ENUMS -----------------------------------------------------------------------

// STRUCTS ---------------------------------------------------------------------

// GLOBAL PARAMETERS -----------------------------------------------------------

// FUNCTION DECLARATION --------------------------------------------------------

// MAIN PROGRAM ----------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_command_request");
  ros::NodeHandle node("marker_command_request");

  int num_tries = 10;

  // Start service client
  ros::ServiceClient client = 
    node.serviceClient<hsr::MarkerCommand>("/marker_req");

  // Try to request service 
  hsr::MarkerCommand mc;
  mc.request.marker_action = "ADD";

  for(int i = 0; i < num_tries; ++i) {
    if(client.call(mc)) {
      ROS_INFO("Marker command response: %s", mc.response.feedback_msg.c_str());
      return EXIT_SUCCESS;
    } else {
      ROS_WARN("ServiceClient failed on attempt %d / %d", i, num_tries);
    }
    usleep(10000);
  }
	return EXIT_SUCCESS;
}

// FUNCTION IMPLEMENTATION -----------------------------------------------------