#include "rgbd/Server.h"

#include <ros/init.h>
#include <ros/rate.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_transport_test_server");

    rgbd::Server server;
    server.initialize("test");

    ros::Rate r(30);

    int x = 0;
    while (ros::ok())
    {
        cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0,0,255));
        cv::line(rgb_image, cv::Point(x, 0), cv::Point(x, rgb_image.rows - 1), cv::Scalar(255, 0, 0));

        cv::Mat depth_image(480, 640, CV_32FC1, 5.0);
        cv::line(depth_image, cv::Point(x, 0), cv::Point(x, depth_image.rows - 1), 1.0);

        geo::DepthCamera cam_model;
        rgbd::Image image(rgb_image, depth_image, cam_model, "test_frame_id", ros::Time::now().toSec());

        server.send(image, true);

        x = (x + 10) % rgb_image.cols;

        r.sleep();
    }

    ros::spin();

    return 0;
}
