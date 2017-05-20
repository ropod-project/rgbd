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

void Client::initialize(const std::string& server_name)
{
  this->intializeROS(server_name + "/rgb/image_rect_color",
                     server_name + "/depth_registered/sw_registered/image_rect_raw",
                     server_name + "/rgb/camera_info");
}

// ----------------------------------------------------------------------------------------

void Client::intializeROS(const std::string& rgb_image_topic, const std::string& depth_image_topic, const std::string& cam_info_topic)
{
  ros_image_sync_data_.sub_cam_info_ = nh_.subscribe(cam_info_topic, 1, &Client::camInfoCallback, this);

  ros_image_sync_data_.sub_rgb_sync_ = ImageSubPtr(new message_filters::Subscriber<sensor_msgs::Image>(nh_, rgb_image_topic, 1));
  ros_image_sync_data_.sub_depth_sync_ = ImageSubPtr(new message_filters::Subscriber<sensor_msgs::Image>(nh_, depth_image_topic, 1));

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
  if (!ros_image_sync_data_.cam_model_.initialized())
    return;

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

void Client::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
  if (!ros_image_sync_data_.cam_model_.initialized())
    ros_image_sync_data_.cam_model_.fromCameraInfo(cam_info_msg);
}

}
