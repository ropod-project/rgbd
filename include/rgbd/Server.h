#ifndef RGBD_TRANSPORT_SERVER_H_
#define RGBD_TRANSPORT_SERVER_H_

#include "rgbd/Image.h"

#include <ros/publisher.h>
#include <ros/callback_queue.h>
#include <ros/service_server.h>

#include <boost/thread.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

typedef boost::shared_ptr<image_transport::ImageTransport> ImageTransportPtr;

namespace rgbd {

class Image;

class Server {

public:

    Server();

    virtual ~Server();

    void initialize(const std::string& name);

    void send(const Image& image, bool threaded = false);

    const static int SERIALIZATION_VERSION;

protected:

    ros::NodeHandle nh_;
    ImageTransportPtr rgb_image_transport_, depth_image_transport_;
    image_transport::CameraPublisher rgb_publisher_, depth_publisher_;

};

}

#endif
