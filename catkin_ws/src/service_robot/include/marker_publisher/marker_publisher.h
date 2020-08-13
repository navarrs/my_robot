#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include "hsr/hsr.h"

namespace hsr { namespace service_robot {

class MarkerPublisher 
{
public:
  /**
   * Constructor
   * */
  MarkerPublisher();
  /**
   * Destructor 
   * */
  ~MarkerPublisher();
  /**
   * Initialize the marker publisher
   * @return Exit code. 
   * */
  Exit Initialize();
private:
  /**
   * Node handle for the marker publisher.
   * */
  ros::NodeHandle node_;
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
};
} // End of namespace service_robot. 
} // End of namespace hsr.