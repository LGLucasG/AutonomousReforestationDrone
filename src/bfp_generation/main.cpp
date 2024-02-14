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

using namespace std;

int main()
{
    if (filesystem::exists("../mask.png"))
    {
        remove("../mask.png");
    }

    for (const filesystem::__cxx11::directory_entry &entry : filesystem::directory_iterator("../"))
    {
        string path = entry.path();
        if (path.find(".png") != string::npos)
        {
            vector<cv::Point> bfp;
            cout << path << endl;
            create_mask(path);
            if (!filesystem::exists("../mask.png"))
            {
                cout << "No file mask.png created. Please retry." << endl;
                return 0;
            }
            vector<cv::Point> imgPoints;
            while (imgPoints.size() != 2)
            {
                imgPoints = get_image_points(path);
            }
            bfp = compute_bfp("../mask.png", DENSITY, SPACING_INTERVAL, DISPLAY);
            if (bfp.size() == 0)
            {
                cout << "No BFP found / bad polygon, please retry." << endl;
                if (filesystem::exists("../mask.png"))
                {
                    remove("../mask.png");
                }
                break;
            }
            cout << "number of points for bfp : " << bfp.size() << endl;
            vector<GPSPoint> gpsPoints = get_gps_points("../gps_keypoints.csv");
            vector<GPSPoint> bfp_gps = transformToGPS(bfp, imgPoints[0], imgPoints[1], gpsPoints[0], gpsPoints[1]);
            ofstream myfile;
            myfile.open("../bfp_gps.csv");
            myfile << "latitude,longitude\n";
            for (int i = 0; i < bfp_gps.size(); i++)
            {
                myfile << setprecision(15) << bfp_gps[i].latitude << "," << bfp_gps[i].longitude << "\n";
            }
            myfile.close();
            cout << "Coordinates of the BFP stored in bfp_gps.csv" << endl;
            break;
        }
    }
    if (filesystem::exists("../mask.png"))
    {
        remove("../mask.png");
    }
    return 0;
}
