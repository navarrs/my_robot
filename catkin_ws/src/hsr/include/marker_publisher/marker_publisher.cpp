#include "marker_publisher.h"

namespace hsr { namespace service_robot {

MarkerPublisher::MarkerPublisher(const ros::NodeHandle &marker_node)
: marker_node_(marker_node) {  

}

MarkerPublisher::~MarkerPublisher() {
  /** Empty Body **/
}

Exit MarkerPublisher::Initialize() {
  // Get all params from launch file
  if(!marker_node_.getParam(param_queue_size_, queue_size_)) {
    ROS_WARN("Unspecified param %s. Default value is %d", 
      param_queue_size_.c_str(), queue_size_);
  }

  // Marker publisher
  std::string param_value = "";
  if(!marker_node_.getParam(topic_marker_pub_, param_value)) {
    ROS_ERROR("Unspecified parameter %s", topic_marker_pub_.c_str());
    return FAIL;
  }
  marker_pub_ = marker_node_.advertise<visualization_msgs::Marker>(
    param_value, queue_size_);

  // Marker pose request 
  if(!marker_node_.getParam(topic_marker_ssrv_, param_value)) {
    ROS_ERROR("Unspecified parameter %s", topic_marker_ssrv_.c_str());
    return FAIL;
  }
  marker_ssrv_ = marker_node_.advertiseService(
    param_value, &MarkerPublisher::OnMarkerCommandRequest, this);
  
  // Wait to get a subscriber to the markers
  while(marker_pub_.getNumSubscribers() < 1) {
    if(!ros::ok()) {
      return FAIL;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  // Set marker
  if(!marker_node_.getParam("/marker/namespace", marker_.ns)) {
    ROS_ERROR("Could not get marker/namespace");
    return FAIL;
  }

  if(!marker_node_.getParam("/marker/frame_id", marker_.header.frame_id)) {
    ROS_ERROR("Could not get marker/frame_id");
    return FAIL;
  }

  if(!marker_node_.getParam("/marker/id", marker_.id)) {
    ROS_WARN("Could not get /marker/id. Default value is 0");
    marker_.id = 0;
  }

  std::vector<double> v;
  if(!marker_node_.getParam("/marker/color", v)) {
    ROS_WARN("Could not get /marker/color. Default is 1.0,1.0,1.0,1.0");
    marker_.color.r = marker_.color.g = marker_.color.g = marker_.color.a = 1.0;
  } else {
    marker_.color.r = v[0];
    marker_.color.g = v[1];
    marker_.color.b = v[2];
    marker_.color.a = v[3];
    v.clear();
  }

  if(!marker_node_.getParam("/marker/scale", v)) {
    ROS_WARN("Could not get /marker/color. Default is 1.0,1.0,1.0,1.0");
    marker_.color.r = marker_.color.g = marker_.color.g = marker_.color.a = 1.0;
  } else {
    marker_.scale.x = v[0];
    marker_.scale.y = v[1];
    marker_.scale.z = v[2];
  }

  if(!marker_node_.getParam("/marker/type", marker_.type)) {
    ROS_WARN("Could not get /marker/type. Default is CUBE");
    marker_.type = visualization_msgs::Marker::CUBE;
  }
  marker_.header.stamp = ros::Time::now();
  marker_.lifetime = ros::Duration();

  ROS_INFO("Marker publisher is ready! Marker Namespace:%s and Frame ID:%s", 
    marker_.ns.c_str(), marker_.header.frame_id.c_str());
  return SUCCESS;
}

bool MarkerPublisher::OnMarkerCommandRequest(
  hsr::MarkerCommand::Request &req, hsr::MarkerCommand::Response &res) {

  if(MarkerAction::Name2Id.find(req.marker_action) == 
     MarkerAction::Name2Id.end()) {
    res.feedback_msg = "Invalid specified marker action " + req.marker_action;
    ROS_ERROR("%s", res.feedback_msg.c_str());
    return false;
  }
  marker_action_ = MarkerAction::Name2Id.find(req.marker_action)->second;
  marker_.pose = req.pose;
  
  switch(marker_action_) {
    case MarkerAction::Id::ADD:
      marker_.action = visualization_msgs::Marker::ADD;
      break;
    case MarkerAction::Id::DELETE:
      marker_.action = visualization_msgs::Marker::DELETE;
      break;
    case MarkerAction::Id::NONE:
    default:
      ROS_WARN("Unsuported marker action %s", req.marker_action.c_str());
      break;
  }

  // Publish the marker
  if(MarkerAction::Id::NONE != marker_action_) {
    marker_pub_.publish(marker_);
    ROS_INFO("Published marker!");
  }

  res.feedback_msg = "MarkerCommandRequest with action " + req.marker_action;
  ROS_INFO("%s", res.feedback_msg.c_str());
  return true;  
}

} // End of namespace service_robot.
} // End of namespace hsr.