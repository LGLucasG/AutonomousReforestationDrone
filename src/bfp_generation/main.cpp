#include <opencv2/opencv.hpp>
#include <iostream>
#include "bfp.hpp"
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>

#define DENSITY 10
#define SPACING_INTERVAL 30
#define DISPLAY 1

int main()
{
    if (std::filesystem::exists("../mask.png"))
    {
        std::remove("../mask.png");
    }

    for (const std::filesystem::__cxx11::directory_entry &entry : std::filesystem::directory_iterator("../"))
    {
        std::string path = entry.path();
        if (path.find(".png") != std::string::npos)
        {
            std::vector<cv::Point> bfp;
            std::cout << path << std::endl;
            create_mask(path);
            bfp = compute_bfp("../mask.png", DENSITY, SPACING_INTERVAL, DISPLAY);
            if (bfp.size() == 0)
            {
                std::cout << "No BFP found / bad polygon, please retry." << std::endl;
                if (std::filesystem::exists("../mask.png"))
                {
                    std::remove("../mask.png");
                }
                break;
            }
            std::cout << "number of points for bfp : " << bfp.size() << std::endl;
            std::vector<cv::Point> imgPoints = get_image_points(path);
            std::vector<GPSPoint> gpsPoints = get_gps_points();
            std::vector<GPSPoint> bfp_gps = transformToGPS(bfp, imgPoints[0], imgPoints[1], gpsPoints[0], gpsPoints[1]);
            std::ofstream myfile;
            myfile.open("../bfp_gps.csv");
            myfile << "latitude,longitude\n";
            for (int i = 0; i < bfp_gps.size(); i++)
            {
                myfile << std::setprecision(15) << bfp_gps[i].latitude << "," << bfp_gps[i].longitude << "\n";
            }
            myfile.close();
            std::cout << "Coordinates of the BFP stored in bfp_gps.csv" << std::endl;
            break;
        }
    }
    if (std::filesystem::exists("../mask.png"))
    {
        std::remove("../mask.png");
    }
    return 0;
}
