/**
 * This client can either be used to listen to RGBD messages on a single topic
 * (use initialize(_)) or to seperate RGB / Depth / camera info topics
 * (use initialize(_, _, _)).
 *
 * The client will always convert the incoming data to rgbd::Image data structures.
 */

#ifndef RGBD_TRANSPORT_CLIENT_H_
#define RGBD_TRANSPORT_CLIENT_H_

#include <ros/ros.h>

#include "rgbd/types.h"

// ROS message serialization
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/subscriber_filter.h>

namespace rgbd {

typedef boost::shared_ptr<image_transport::ImageTransport> ImageTransportPtr;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> KinectApproxPolicy;
typedef boost::shared_ptr<message_filters::Synchronizer<KinectApproxPolicy> > SyncPtr;
typedef boost::shared_ptr<image_transport::SubscriberFilter> ImageSubPtr;

struct ROSImageSyncData
{
  ImageSubPtr sub_rgb_sync_;
  ImageSubPtr sub_depth_sync_;
  SyncPtr sync_;
  image_geometry::PinholeCameraModel cam_model_;
};

class Image;

class Client {

public:

    Client();

    virtual ~Client();

    void initialize(const std::string& server_name, bool compressed = false);

    bool initialized() { return ros_image_sync_data_.sync_.get(); }

    ImagePtr nextImage();

    bool nextImage(Image& image);

protected:

    void intializeROS(bool compressed);

    ros::NodeHandle nh_;

    bool received_image_;
    ImagePtr image_ptr_;

    ROSImageSyncData ros_image_sync_data_;
    ImageTransportPtr rgb_image_transport_, depth_image_transport_;

    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void imageCallback(sensor_msgs::ImageConstPtr rgb_image_msg, sensor_msgs::ImageConstPtr depth_image_msg);

};

}

#endif
