#include "rgbd/Client.h"
#include "rgbd/Image.h"

#include <rgbd/serialization.h>
#include <fstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgbd_saver");

  std::string file_name = "rgbd_image";

  rgbd::Client client;
  client.initialize("rgbd");

  ros::Rate r(30);
  while (ros::ok()) {
    rgbd::Image image;
    if (client.nextImage(image))
    {
      // write
      std::ofstream f_out;
      f_out.open(file_name.c_str(), std::ifstream::binary);
      tue::serialization::OutputArchive a_out(f_out);

      rgbd::serialize(image, a_out, rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_PNG);
      f_out.close();

      ROS_INFO("Image stored to disk.");
      break;
    }
    else
    {
      ROS_INFO("No image received ...");
    }

    r.sleep();
  }

  return 0;
}
