#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  if (argc <= 1)
  {
      std::cout << "Please provide rgbd topic" << std::endl;
      return 1;
  }

  ros::init(argc, argv, "rgbd_viewer");

  rgbd::Client client;
  client.initialize(argv[1]);

  ros::Rate r(30);
  while (ros::ok())
  {
    rgbd::Image image;
    if (client.nextImage(image))
    {
      // Show depth image
      if (image.getDepthImage().data)
        cv::imshow("depth_original", image.getDepthImage() / 8);

      // Show rgb image
      if (image.getRGBImage().data)
        cv::imshow("rgb_original", image.getRGBImage());

      // Show combined image
      if (image.getDepthImage().data && image.getRGBImage().data)
      {
        cv::Mat image_hsv;
        cv::cvtColor(image.getRGBImage(), image_hsv, CV_BGR2HSV);

        rgbd::View view(image, image_hsv.cols);

        cv::Mat canvas_hsv(view.getHeight(), view.getWidth(), CV_8UC3, cv::Scalar(0, 0, 0));

        for(int y = 0; y < view.getHeight(); ++y)
        {
          for(int x = 0; x < view.getWidth(); ++x)
          {
            float d = view.getDepth(x, y);
            if (d == d)
            {
              cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(y, x);
              hsv[2] = 255 - (d / 8 * 255);
              canvas_hsv.at<cv::Vec3b>(y, x) = hsv;
            }
          }
        }

        cv::Mat canvas_bgr;
        cv::cvtColor(canvas_hsv, canvas_bgr, CV_HSV2BGR);
        cv::imshow("image + depth", canvas_bgr);
      }

      cv::waitKey(3);
    }

    r.sleep();
  }

  ros::spin();

  return 0;
}
