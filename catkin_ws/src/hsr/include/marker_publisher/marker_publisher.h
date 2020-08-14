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

// DEFINES ---------------------------------------------------------------------

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace hsr { namespace service_robot {
// FOWARD DECLARATIONS ---------------------------------------------------------

// ENUMS -----------------------------------------------------------------------

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
   * Current marker to publish.
   * */
  visualization_msgs::Marker marker_;
  /**
   * Pose where to publish a the marker.
   * */
  geometry_msgs::Pose marker_pose_;
  /**
   * Topic to publish markers.
   * */
  std::string marker_pub_topic_ = "visualization_marker";
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
};
} // End of namespace service_robot. 
} // End of namespace hsr.
#endif