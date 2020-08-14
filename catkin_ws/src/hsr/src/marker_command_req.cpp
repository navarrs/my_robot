/** ----------------------------------------------------------------------------
 * @file: marker_command_req.cpp
 * @date: August 14, 2020
 * @author: navarrs
 * 
 * @brief: Requests a marker action 
 * ---------------------------------------------------------------------------*/

// INCLUDES --------------------------------------------------------------------
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  ros::init(argc, argv, "marker_command");
  ros::NodeHandle node("marker_command");

  std::string param_value;
  if(!node.getParam("marker_csrv", param_value)) {
    ROS_ERROR("Could not get parameter marker_csrv");
    return EXIT_FAILURE;
  }
  ros::ServiceClient client = 
    node.serviceClient<hsr::MarkerCommand>(param_value);

  int num_tries = 10;
  if(!node.getParam("num_tries", num_tries)) {
    ROS_WARN("Could not get number of tries. Default is %d", num_tries);
  }

  // Set marker command
  hsr::MarkerCommand mc;
  if(!node.getParam("marker_action", mc.request.marker_action)) {
    ROS_ERROR("Unspecified marker action");
    return EXIT_FAILURE;
  }
  ROS_INFO("MarkerAction requested: %s", mc.request.marker_action.c_str());

  // Get pose 
  geometry_msgs::Pose pose;
  node.getParam("x", pose.position.x);
  node.getParam("y", pose.position.y);
  node.getParam("z", pose.position.z);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  node.getParam("roll", roll);
  node.getParam("pitch", pitch);
  node.getParam("yaw", yaw);
  ROS_INFO("MarkerPose:\nx:%lf, y:%lf, z:%lf\nroll:%lf, pitch:%lf, yaw:%lf", 
    pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);

  // Convert orientation from euler angles to quaternion
  tf2::Quaternion q;
  q.setEuler(yaw, pitch, roll);
  pose.orientation = tf2::toMsg(q);
  mc.request.pose = pose;

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