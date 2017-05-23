#include "rgbd/Server.h"
#include "rgbd/serialization.h"
#include "rgbd/View.h"
#include "rgbd/ros/conversions.h"
#include "rgbd/Image.h"

#include <opencv2/highgui/highgui.hpp>

#include <tue/serialization/conversions.h>

#include <tue/profiling/timer.h>

#include <ros/node_handle.h>

namespace rgbd {

const int Server::SERIALIZATION_VERSION = 2;

// ----------------------------------------------------------------------------------------

Server::Server()
{
}

// ----------------------------------------------------------------------------------------

Server::~Server()
{
}

// ----------------------------------------------------------------------------------------

void Server::initialize(const std::string& name)
{
  nh_ = ros::NodeHandle(name);
  rgb_image_transport_ = ImageTransportPtr(new image_transport::ImageTransport(ros::NodeHandle(nh_, "rgb")));
  depth_image_transport_ = ImageTransportPtr(new image_transport::ImageTransport(ros::NodeHandle(nh_, "depth_registered/sw_registered")));
  rgb_publisher_ = rgb_image_transport_->advertiseCamera("image_rect_color", 1, true);
  depth_publisher_ = depth_image_transport_->advertiseCamera("image_rect", 1, true);
}

// ----------------------------------------------------------------------------------------

void Server::send(const Image& image, bool threaded)
{
  Image image_copy = image.clone();

  std_msgs::Header header;
  header.frame_id = image_copy.getFrameId();
  header.stamp.fromSec(image_copy.getTimestamp());

  // Convert camera info to message
  rgbd::View view(image, image.getDepthImage().cols);
  sensor_msgs::CameraInfo info_msg;
  rgbd::convert(view.getRasterizer(), info_msg);
  info_msg.header = header;

  sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(header, "bgr8", image_copy.getRGBImage()).toImageMsg();

  // Convert image to message
  sensor_msgs::Image depth_msg;
  rgbd::convert(image_copy.getDepthImage(), depth_msg);
  depth_msg.header.stamp = ros::Time(image_copy.getTimestamp());
  depth_msg.header.frame_id = image_copy.getFrameId();

  rgb_publisher_.publish(*rgb_msg, info_msg);
  depth_publisher_.publish(depth_msg, info_msg);
}

// ----------------------------------------------------------------------------------------

}
