#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

bool PAUSE = false;
std::string MODE;

std::vector<cv::Vec2i> mouse_points;

// ----------------------------------------------------------------------------------------------------

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
         mouse_points.push_back(cv::Vec2i(x, y));
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
//          std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
     }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_multitool");

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
            client->intialize(arg);
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
                    int width = std::min(rgb.cols, depth.cols);
                    int height = std::min(rgb.rows, depth.rows);

                    canvas = cv::Mat(height, width * 2, CV_8UC3);

                    cv::Mat roi1 = canvas(cv::Rect(cv::Point(0, 0), cv::Size(width, height)));
                    cv::Mat roi2 = canvas(cv::Rect(cv::Point(width, 0), cv::Size(width, height)));

                    rgb.copyTo(roi1);
                    depth_canvas.copyTo(roi2);
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

        cv::putText(canvas, MODE, cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

        if (PAUSE)
            cv::putText(canvas, "PAUSED", cv::Point(10, canvas.rows - 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        for(unsigned int i = 0; i < mouse_points.size(); ++i)
        {
            cv::circle(canvas, mouse_points[i], 5, cv::Scalar(0, 0, 255), 2);
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
