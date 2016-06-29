#include <rgbd/serialization.h>
#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <fstream>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

    if (argc < 2)
    {
        std::cout << "Usage:\n\n   rgbd_to_rgb_png FILENAME\n\n";
        return 1;
    }

    for (unsigned int i = 1; i < argc; ++i)
    {
        std::string name = std::string(argv[i]);

        // read
        std::ifstream f_in;
        f_in.open(name.c_str(), std::ifstream::binary);

        if (!f_in.is_open())
        {
            std::cerr << "Could not open '" << name << "'." << std::endl;
            continue;
        }

        tue::serialization::InputArchive a_in(f_in);

        rgbd::Image image;
        rgbd::deserialize(a_in, image);


        size_t lastindex = name.find_last_of(".");
        name = name.substr(0, lastindex);

        std::string rgb_filename = name + "_rgb.png";

        if (cv::imwrite(rgb_filename, image.getRGBImage()))
            std::cout << "Succesfully stored '" << rgb_filename << "'" << std::endl;
        else
            std::cerr << "Failed to write rgbd to png" << std::endl;
    }

    return 0;
}
