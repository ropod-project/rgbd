#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

bool PAUSE = false;
std::string MODE;

int IMAGE_WIDTH, IMAGE_HEIGHT;

std::vector<cv::Vec2i> mouse_points;
cv::Vec2i mouse_pos;

// ----------------------------------------------------------------------------------------------------

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    x = x % IMAGE_WIDTH;
    mouse_pos = cv::Vec2i(x, y);

    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        mouse_points.push_back(mouse_pos);
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        //          std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        //          std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
//        mouse_pos = cv::Vec2i(x, y);
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_multitool", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    rgbd::Client* client = 0;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Parse command line arguments

    if (argc < 3)
    {
        std::cout << "Usage:" << std::endl
                  << std::endl
                  << "    multitool --rgbd RGBD_TOPIC" << std::endl
                  << std::endl;
        return 1;
    }

    for(unsigned int i = 1; i < argc; i += 2)
    {
        std::string opt = argv[i];
        std::string arg = argv[i + 1];

        if (opt == "--rgbd")
        {
            client = new rgbd::Client;
            client->initialize(arg, true);
        }
        else
        {
            std::cout << "Unknown option: '" << opt << "'." << std::endl;
            return 1;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::cout << "Keys:" << std::endl
              << std::endl
              << "    spacebar - Pause" << std::endl
              << "    m        - Measure" << std::endl
              << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    //Create a window
    cv::namedWindow("RGBD", 1);

    //set the callback function for any mouse event
    cv::setMouseCallback("RGBD", CallBackFunc, NULL);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    float max_view_distance = 10;

    rgbd::ImagePtr image;

    ros::Rate r(30);
    while (ros::ok())
    {
        if (!PAUSE && client)
        {
            rgbd::ImagePtr image_tmp = client->nextImage();
            if (image_tmp)
                image = image_tmp;
        }

        cv::Mat canvas;
        IMAGE_WIDTH = 0;
        IMAGE_HEIGHT = 0;

        if (image)
        {
            const cv::Mat& rgb = image->getRGBImage();
            const cv::Mat& depth = image->getDepthImage();

            if (depth.data)
            {
                cv::Mat depth_canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(50, 0, 0));
                for(int y = 0; y < depth.rows; ++y)
                {
                    for(int x = 0; x < depth.cols; ++x)
                    {
                        float d = depth.at<float>(y, x);
                        if (d > 0 && d == d)
                        {
                            unsigned char v = std::min<float>(max_view_distance, d / max_view_distance) * 255;
                            depth_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(v, v, v);
                        }
                    }
                }

                if (rgb.data)
                {
                    IMAGE_WIDTH = std::min(rgb.cols, depth.cols);

                    int rgb_height = IMAGE_WIDTH * rgb.rows / rgb.cols;
                    int depth_height = IMAGE_WIDTH * depth.rows / depth.cols;

                    IMAGE_HEIGHT = std::max(rgb_height, depth_height);

                    canvas = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH * 2, CV_8UC3, cv::Scalar(50, 50, 50));

                    cv::Mat rgb_roi = canvas(cv::Rect(cv::Point(0, 0), cv::Size(IMAGE_WIDTH, rgb_height)));
                    cv::Mat depth_roi = canvas(cv::Rect(cv::Point(IMAGE_WIDTH, 0), cv::Size(IMAGE_WIDTH, depth_height)));

                    cv::resize(rgb, rgb_roi, cv::Size(IMAGE_WIDTH, rgb_height));
                    cv::resize(depth_canvas, depth_roi, cv::Size(IMAGE_WIDTH, depth_height));
                }
                else
                {
                    canvas = depth_canvas;
                }
            }
            else if (rgb.data)
            {
                canvas = rgb;
            }
        }

        if (!canvas.data)
        {
            canvas = cv::Mat(480, 640, CV_8UC3, cv::Scalar(50, 50, 50));
            cv::line(canvas, cv::Point(0, 0), cv::Point(640, 480), cv::Scalar(255, 255, 255), 5);
            cv::line(canvas, cv::Point(0, 480), cv::Point(640, 0), cv::Scalar(255, 255, 255), 5);
        }

        if (IMAGE_WIDTH == 0 || IMAGE_HEIGHT == 0)
        {
            IMAGE_WIDTH = canvas.cols;
            IMAGE_HEIGHT = canvas.rows;
        }

        // Show mouse cursor(s)
        cv::circle(canvas, mouse_pos, 5, cv::Scalar(255, 0, 0), 1);
        if (canvas.cols > IMAGE_WIDTH)
            cv::circle(canvas, mouse_pos + cv::Vec2i(IMAGE_WIDTH, 0), 5, cv::Scalar(255, 0, 0), 1);

        cv::putText(canvas, MODE, cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

        if (PAUSE)
            cv::putText(canvas, "PAUSED", cv::Point(10, canvas.rows - 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        for(unsigned int i = 0; i < mouse_points.size(); ++i)
        {
            cv::circle(canvas, mouse_points[i], 5, cv::Scalar(0, 0, 255), 2);
            if (canvas.cols > IMAGE_WIDTH)
                cv::circle(canvas, mouse_points[i] + cv::Vec2i(IMAGE_WIDTH, 0), 5, cv::Scalar(0, 0, 255), 2);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        if (MODE == "MEASURE")
        {
            if (mouse_points.size() == 2)
            {
                rgbd::View view(*image, 640);

                geo::Vector3 p1, p2;

                if (view.getPoint3D(mouse_points[0][0], mouse_points[0][1], p1) && view.getPoint3D(mouse_points[1][0], mouse_points[1][1], p2))
                {
                    std::cout << (p1 - p2).length() << " m" << std::endl;
                }

                mouse_points.clear();
            }
        }
        else
        {
            mouse_points.clear();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        cv::imshow("RGBD", canvas);
        int i_key = cv::waitKey(3);
        if (i_key >= 0)
        {
            char key = i_key;

            switch (key)
            {
            case ' ': PAUSE = !PAUSE;
                break;
            case 'm': MODE = "MEASURE";
                break;
            default: MODE = "";
                break;
            }
        }

        r.sleep();
    }

    ros::spin();

    return 0;
}
