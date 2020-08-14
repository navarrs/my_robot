/** ----------------------------------------------------------------------------
 * @file: marker_publisher.h
 * @date: August 13, 2020
 * @author: navarrs
 * 
 * @brief: This is the header file for the MarkerPublisher class which is used
 * to publish visual markers on RVIZ. 
 * ---------------------------------------------------------------------------*/
// IFNDEF ----------------------------------------------------------------------
#ifndef HSR_MARKER_PUBLISHER_H_
#define HSR_MARKER_PUBLISHER_H_

// INCLUDES --------------------------------------------------------------------
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include "hsr/hsr.h"
#include "hsr/MarkerCommand.h"

// DEFINES ---------------------------------------------------------------------

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace hsr { namespace service_robot {
// FOWARD DECLARATIONS ---------------------------------------------------------

// ENUMS -----------------------------------------------------------------------
namespace MarkerAction {
  enum class Id {
    NONE = 0, ADD, DELETE
  };
  std::map<std::string, Id> Name2Id = {
    {"NONE", Id::NONE}, {"none", Id::NONE},
    {"ADD", Id::ADD}, {"add", Id::ADD},
    {"DELETE", Id::DELETE}, {"delete", Id::DELETE},
  }; 
} // End of namespace MarkerAction 
// STRUCTS ---------------------------------------------------------------------

// TYPEDEFS --------------------------------------------------------------------

// CLASS DECLARATION -----------------------------------------------------------
class MarkerPublisher 
{
public:
  // FUNCTIONS -----------------------------------------------------------------
  /**
   * Constructor
   * @param marker_node Node Handle to publish markers. 
   * */
  MarkerPublisher(const ros::NodeHandle &marker_node);
  /**
   * Destructor 
   * */
  ~MarkerPublisher();
  /**
   * Initialize the marker publisher
   * @return Exit code. 
   * */
  Exit Initialize();
  // MEMBER GETTERS ------------------------------------------------------------

  // MEMBER SETTERS ------------------------------------------------------------

  // MEMBER GETTERS ------------------------------------------------------------
private:
  // FUNCTIONS -----------------------------------------------------------------
  /**
   * Handles marker command requests. 
   * @param req message request.
   * @param res message response.
   * @return true if successful, false otherwise
   * */
  bool OnMarkerCommandRequest(
    hsr::MarkerCommand::Request &req, hsr::MarkerCommand::Response &res);

  // PRIVATE MEMBERS -----------------------------------------------------------
  /**
   * Node handle for the marker publisher.
   * */
  ros::NodeHandle marker_node_;
  /**
   * Marker publisher.
   * */
  ros::Publisher marker_pub_;
  /**
   * Marker request service server.
   * */
  ros::ServiceServer marker_ssrv_;
  /**
   * Current marker to publish.
   * */
  visualization_msgs::Marker marker_;
  /**
   * Pose where to publish a the marker.
   * */
  geometry_msgs::Pose marker_pose_;
  /**
   * Current marker action 
   * */
  MarkerAction::Id marker_action_ = MarkerAction::Id::NONE;
  /**
   * Topic to publish markers.
   * */
  std::string marker_pub_topic_ = "marker_visualization";
  /**
   * Namespace of marker.
   * */
  std::string marker_namespace_ = "marker_publisher";
  /**
   * Message queue size
   * */
  int queue_size_ = 10;

  // LAUNCH PARAMETERS AND TOPICS ----------------------------------------------
  const std::string param_queue_size_ = "queue_size";
  const std::string topic_marker_pub_ = "marker_pub";
  const std::string topic_marker_ssrv_ = "marker_ssrv";
};
} // End of namespace service_robot. 
} // End of namespace hsr.
#endif