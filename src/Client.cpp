#include "rgbd/Client.h"
#include <opencv2/highgui/highgui.hpp>
#include "rgbd/Image.h"

// RGBD message serialization
#include "rgbd/serialization.h"
#include <tue/serialization/conversions.h>

namespace rgbd {

// ----------------------------------------------------------------------------------------

Client::Client()
{
}

// ----------------------------------------------------------------------------------------

Client::~Client()
{
}

// ----------------------------------------------------------------------------------------

void Client::initialize(const std::string& server_name, bool compressed)
{
  nh_ = ros::NodeHandle(server_name);
  this->intializeROS(compressed);
}

// ----------------------------------------------------------------------------------------

void Client::intializeROS(bool compressed)
{
  rgb_image_transport_ = ImageTransportPtr(new image_transport::ImageTransport(ros::NodeHandle(nh_, "rgb")));
  depth_image_transport_ = ImageTransportPtr(new image_transport::ImageTransport(ros::NodeHandle(nh_, "depth_registered/sw_registered")));

  if (compressed)
  {
    ros_image_sync_data_.sub_rgb_sync_ = ImageSubPtr(new image_transport::SubscriberFilter(*rgb_image_transport_, "image_rect_color", 1, image_transport::TransportHints("compressed")));
    ros_image_sync_data_.sub_depth_sync_ = ImageSubPtr(new image_transport::SubscriberFilter(*depth_image_transport_, "image_rect", 1, image_transport::TransportHints("compressedDepth")));
  }
  else
  {
    ros_image_sync_data_.sub_rgb_sync_ = ImageSubPtr(new image_transport::SubscriberFilter(*rgb_image_transport_, "image_rect_color", 1));
    ros_image_sync_data_.sub_depth_sync_ = ImageSubPtr(new image_transport::SubscriberFilter(*depth_image_transport_, "image_rect", 1));
  }

  ros_image_sync_data_.sync_ = SyncPtr(new message_filters::Synchronizer<KinectApproxPolicy>
                                       (KinectApproxPolicy(10), *ros_image_sync_data_.sub_rgb_sync_,
                                        *ros_image_sync_data_.sub_depth_sync_));
  ros_image_sync_data_.sync_->registerCallback(boost::bind(&Client::imageCallback, this, _1, _2));
}

// ----------------------------------------------------------------------------------------

ImagePtr Client::nextImage()
{
  image_ptr_.reset();
  ros::spinOnce();
  return image_ptr_;
}

bool Client::nextImage(Image& image)
{
  ImagePtr next_image = nextImage();
  if (next_image) {
    image = *next_image;
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

void Client::imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg)
{
  cv_bridge::CvImagePtr img_ptr, depth_img_ptr;

  // Convert RGB image
  try
  {
    img_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not deserialize rgb image: %s", e.what());
    return;
  }

  // Convert depth image
  try
  {
    depth_img_ptr = cv_bridge::toCvCopy(depth_image_msg, depth_image_msg->encoding);

    if (depth_image_msg->encoding == "16UC1")
    {
      // depths are 16-bit unsigned ints, in mm. Convert to 32-bit float (meters)
      cv::Mat depth_image(depth_img_ptr->image.rows, depth_img_ptr->image.cols, CV_32FC1);
      for(int x = 0; x < depth_image.cols; ++x)
      {
        for(int y = 0; y < depth_image.rows; ++y)
        {
          depth_image.at<float>(y, x) = ((float)depth_img_ptr->image.at<unsigned short>(y, x)) / 1000; // (mm to m)
        }
      }

      depth_img_ptr->image = depth_image;
    }

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not deserialize depth image: %s", e.what());
    return;
  }

  image_ptr_ = ImagePtr(new Image(img_ptr->image, depth_img_ptr->image, ros_image_sync_data_.cam_model_, rgb_image_msg->header.frame_id, rgb_image_msg->header.stamp.toSec()));
}

// ----------------------------------------------------------------------------------------

}
