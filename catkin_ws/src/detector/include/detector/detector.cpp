/** 
 * @file: detector.cpp
 * @date: May 29, 2020
 * @author: Ingrid Navarro (navarrs)
 * 
 * @brief: This is the source file for the Detector class. 
 ******************************************************************************/
/**
 * INCLUDES 
 ******************************************************************************/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "detector.h"

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
 * STRUCTS 
 ******************************************************************************/

/**
 * CLASS IMPLEMENTATION 
 ******************************************************************************/
Detector::Detector(const Type::ID &detector_type) 
  : m_detector_type(detector_type) 
{
  /* Empty */
}

Detector::~Detector() 
{
  /* Empty */
}

cv::Mat Detector::ImageConverter(const sensor_msgs::Image &image) 
{
	cv_bridge::CvImagePtr cv_ptr;
	try 
  {
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	} 
  catch(cv_bridge::Exception &e) 
  {
		ROS_ERROR("Could not convert image. Exception: %s", e.what());
		return cv::Mat{};
	}
	return cv_ptr->image;
}

cv::Mat Detector::GetImage() const 
{
	return m_image;
}

void Detector::EnableDrawDetection(bool on_off) 
{
	m_draw_detections = on_off;
}

} // End of namespace my_detector
} // End of namespace my_robot
/******************************************************************************/