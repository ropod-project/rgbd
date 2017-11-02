#ifndef RGBD_ROS_CONVERSIONS_H_
#define RGBD_ROS_CONVERSIONS_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace cv { class Mat; }
namespace geo { class DepthCamera; }

namespace rgbd
{

bool convert(const cv::Mat& image, sensor_msgs::Image& image_msg);

void convert(const geo::DepthCamera& cam_model, sensor_msgs::CameraInfo& cam_model_msg);

bool convert(const cv::Mat& image, const geo::DepthCamera& cam_model, sensor_msgs::Image& image_msg, sensor_msgs::CameraInfo& cam_model_msg);

}

#endif
