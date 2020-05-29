/** 
 * @file: detector.h
 * @date: May 29, 2020
 * @author: Ingrid Navarro (navarrs)
 * 
 * @brief: This is the header file for the Detector class. 
 ******************************************************************************/

/**
 * IFNDEF 
 ******************************************************************************/
#ifndef MY_ROBOT_DETECTOR_H
#define MY_ROBOT_DETECTOR_H

/**
 * DEFINES 
 ******************************************************************************/

/**
 * INCLUDES 
 ******************************************************************************/
#include <geometry_msgs/Pose2D.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <common/common.h>
#include "detector/Detect.h"

/**
 * FOWARD DECLARATIONS 
 ******************************************************************************/

/**
 * NAMESPACES 
 ******************************************************************************/
namespace my_robot { 
namespace my_detector {

/**
 * ENUMS 
 ******************************************************************************/
/**
 * Type of detector to launch.
 * */
namespace Type 
{
	enum class ID 
  { 
		DETECTOR = 0, 
		BALL_DETECTOR,
	};
	std::map<ID, std::string> ID2NAME 
  {
		{ID::DETECTOR, "DETECTOR"},
		{ID::BALL_DETECTOR, "BALL_DETECTOR"}
	};
	std::map<std::string, ID> NAME2ID 
  {
		{"DETECTOR", ID::DETECTOR},
		{"BALL_DETECTOR", ID::BALL_DETECTOR}
	};
} // End of namespace Type

/**
 * STRUCTS
 ******************************************************************************/

/**
 * TYPEDEFS 
 ******************************************************************************/

/**
 * CLASS DECLARATION 
 ******************************************************************************/
class Detector 
{
public:
	/**
   * PUBLIC FUNCTIONS 
   ****************************************************************************/
	/**
	 * Constructor. 
	 * @param detector_type type of detector used.
	 * */
	Detector(const Type::ID &detector_type = Type::ID::DETECTOR);
	/**
	 * Destructor.
	 * */
	~Detector();
	/**
	 * Initialize detector.
   * @return exit code.
	 * */
	virtual Exit Initialize() = 0;
	/** 
	 * Callback to process images. 
	 * @param image received image.  
	 * */
	virtual void OnImageMessage(const sensor_msgs::Image &image) = 0;
	/**
	 * Callback that resolves detection requests.
	 * @param req message request.
	 * @param res message response.
	 * @return true, if request was handled successfully. Otherwise false.
	 * */
	virtual bool OnDetectRequest(
		detector::Detect::Request &request, detector::Detect::Response &response) = 0;
	/**
	 * Enables / disables drawing detections.
	 * @param on_off[in]: enable / disable value.
	 * */
	void EnableDrawDetection(bool on_off);
	
	/**
   * MEMBER GETTERS
   ****************************************************************************/
	cv::Mat GetImage() const;

	/**
   * MEMBER SETTERS 
   ****************************************************************************/

  /**
   * PUBLIC MEMBER 
   ****************************************************************************/
protected:
	/**
   * PRIVATE FUNCTIONS 
   ****************************************************************************/
	/**
	 * Converts a ROS image message into an OpenCV message. 
	 * @param image received image.
	 * @return cv::Mat that contains the image
	 * */
	cv::Mat ImageConverter(const sensor_msgs::Image &image);
	/**
	 * Detects an object in an image.
	 * @param location location wrt the image of the detected object. In case that
   * it fails to detect the object of interest, location will be set to -1.
	 * @return exit code.
	 * */
	virtual Exit Detect(geometry_msgs::Pose2D &location) = 0;

	/**
   * PRIVATE MEMBERS
   ****************************************************************************/
	/**
	 *  Detector node
	 * */
	ros::NodeHandle m_detector_node;
	/**
	 * Image subscriber 
	 * */
	ros::Subscriber m_image_sub;
	/**
	 * Publishes location of detected object 
	 * */
	ros::Publisher m_detection_location_pub;
	/**
	 * Service to handle detection requests
	 * */
	ros::ServiceServer m_detect_srv;
	/**
	 * Current image
	 * */
	cv::Mat m_image;
	/**
	 * Detector type
	 * */
	Type::ID m_detector_type = Type::ID::DETECTOR;
	/**
	 * Message queue size
	 * */
	int m_queue_size = 10;
	/**
	 * Draw detections 
	 * */
	bool m_draw_detections = false;
  /**
   * Parameter scopes
   * */
  const std::string m_image_sub_param = "image_sub";
  const std::string m_location_pub_param = "location_pub";
  const std::string m_detect_srv_param = "detect_req";
  const std::string m_message_queue_param = "msg_size";
  const std::string m_camera_model_param = "cam_params";
};

} // End of namespace detector
} // End of namespace my_robot
#endif
/******************************************************************************/