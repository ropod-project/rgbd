#include "rgbd/ros/conversions.h"

#include <opencv2/core/core.hpp>
#include <geolib/sensors/DepthCamera.h>

#include <cv_bridge/cv_bridge.h>

namespace rgbd
{

// ----------------------------------------------------------------------------------------------------

bool convert(const cv::Mat& image, sensor_msgs::Image& image_msg)
{
    cv_bridge::CvImage image_cv_bridge;

    if (image.type() == CV_32FC1) // Depth image
        image_cv_bridge.encoding = "32FC1";
    else if (image.type() == CV_8UC3) // RGB image;
        image_cv_bridge.encoding = "bgr8";
    else
        return false;

    image_cv_bridge.image = image;
    image_cv_bridge.toImageMsg(image_msg);
}

// ----------------------------------------------------------------------------------------------------

void convert(const geo::DepthCamera& cam_model, sensor_msgs::CameraInfo& cam_model_msg)
{
    // Distortion model and parameters
    cam_model_msg.distortion_model = "plumb_bob";
    cam_model_msg.D.resize(5, 0);

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    cam_model_msg.K[0] = cam_model.getFocalLengthX();
    cam_model_msg.K[1] = 0;
    cam_model_msg.K[2] = cam_model.getOpticalCenterX();
    cam_model_msg.K[3] = 0;
    cam_model_msg.K[4] = cam_model.getFocalLengthY();
    cam_model_msg.K[5] = cam_model.getOpticalCenterX();
    cam_model_msg.K[6] = 0;
    cam_model_msg.K[7] = 0;
    cam_model_msg.K[8] = 1;

    // Rectification matrix (stereo cameras only)
    cam_model_msg.R[0] = 1;
    cam_model_msg.R[1] = 0;
    cam_model_msg.R[2] = 0;
    cam_model_msg.R[3] = 0;
    cam_model_msg.R[4] = 1;
    cam_model_msg.R[5] = 0;
    cam_model_msg.R[6] = 0;
    cam_model_msg.R[7] = 0;
    cam_model_msg.R[8] = 1;

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    cam_model_msg.P[0] = cam_model.getFocalLengthX();
    cam_model_msg.P[1] = 0;
    cam_model_msg.P[2] = cam_model.getOpticalCenterX();
    cam_model_msg.P[3] = cam_model.getOpticalTranslationX();
    cam_model_msg.P[4] = 0;
    cam_model_msg.P[5] = cam_model.getFocalLengthY();
    cam_model_msg.P[6] = cam_model.getOpticalCenterY();
    cam_model_msg.P[7] = cam_model.getOpticalTranslationY();
    cam_model_msg.P[8] = 0;
    cam_model_msg.P[9] = 0;
    cam_model_msg.P[10] = 1;
    cam_model_msg.P[11] = 0;

    // TODO: add width and height field to DepthCamera
    cam_model_msg.width = (cam_model.getOpticalCenterX() + 0.5) * 2; // 0.5 instead of 1 to round up properly
    cam_model_msg.height = (cam_model.getOpticalCenterY() + 0.5) * 2;

    cam_model_msg.binning_x = 0;
    cam_model_msg.binning_y = 0;
}

bool convert(const cv::Mat& image, 
             const geo::DepthCamera& cam_model, 
             sensor_msgs::Image& image_msg, 
             sensor_msgs::CameraInfo& cam_model_msg)
{
    convert(cam_model, cam_model_msg);
    size_t width = cam_model_msg.width;
    size_t height = cam_model_msg.height;

    cv_bridge::CvImage image_cv_bridge;

    cv::Mat img_rect;
    if (image.type() == CV_32FC1) { // Depth image 
        image_cv_bridge.encoding = "32FC1";
        img_rect = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));
    }
    else if (image.type() == CV_8UC3)  { // RGB image;
        image_cv_bridge.encoding = "bgr8";
        img_rect = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    }
    else
        return false;

    // Create a ROI for the part of the image that we need to copy
    cv::Rect crop_rect(0, 0, std::min(img_rect.cols, image.cols), std::min(img_rect.rows, image.rows));
    image(crop_rect).copyTo(img_rect.rowRange(0, crop_rect.height).colRange(0, crop_rect.width));

    image_cv_bridge.image = img_rect;
    image_cv_bridge.toImageMsg(image_msg);
}

}
