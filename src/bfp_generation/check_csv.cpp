#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <GeographicLib/LocalCartesian.hpp>

GeographicLib::LocalCartesian gpsToLocalConverter;

void initializeGPSToLocalConverter(double initialLatitude, double initialLongitude, double initialAltitude)
{
	gpsToLocalConverter.Reset(initialLatitude, initialLongitude, initialAltitude);
}

std::tuple<double, double, double> convertGPSToLocal(double latitude, double longitude, double altitude)
{
	double localX, localY, localZ;
	gpsToLocalConverter.Forward(latitude, longitude, altitude, localX, localY, localZ);
	return {localX, localY, localZ};
}

struct Point {
    double x, y;
};

std::vector<Point> parseCSV(const std::string& filename) {
    std::vector<Point> points;
    initializeGPSToLocalConverter(47.39811011441758, 8.545620463903429, 35);

    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string xStr, yStr;

            std::getline(ss, xStr, ',');
            std::getline(ss, yStr, ',');

            Point point;
            point.x = std::stof(xStr);
            point.y = std::stof(yStr);
            std::tuple<double, double, double> convert_point = convertGPSToLocal(point.x, point.y, 0);
            point.x = std::get<0>(convert_point);
            point.y = std::get<1>(convert_point);
            points.push_back(point);
        }

        file.close();
    } else {
        std::cout << "Failed to open the file." << std::endl;
    }
    std::cout << "Number of points: " << points.size() << std::endl;
    return points;
}

// find max and min x
std::pair<double, double> findXMinMax(const std::vector<Point>& points) {
    double minX = points[0].x;
    double maxX = points[0].x;

    for (const auto& point : points) {
        if (point.x < minX) {
            minX = point.x;
        } else if (point.x > maxX) {
            maxX = point.x;
        }
    }
    std::cout << "minX: " << minX << " maxX: " << maxX << std::endl;
    return std::make_pair(minX, maxX);
}

// find max and min y
std::pair<double, double> findYMinMax(const std::vector<Point>& points) {
    double minY = points[0].y;
    double maxY = points[0].y;

    for (const auto& point : points) {
        if (point.y < minY) {
            minY = point.y;
        } else if (point.y > maxY) {
            maxY = point.y;
        }
    }
    std::cout << "minY: " << minY << " maxY: " << maxY << std::endl;
    return std::make_pair(minY, maxY);
}


// create a matrix of pixel using opencv and draw the points
void drawPoints(const std::vector<Point>& points) {
    int coef = 10;
    std::pair<double, double> xMinMax = findXMinMax(points);
    std::pair<double, double> yMinMax = findYMinMax(points);
    double xBound = (xMinMax.second - xMinMax.first)*coef;
    double yBound = (yMinMax.second - yMinMax.first)*coef;
    std::cout << "xBound: " << xBound << " yBound: " << yBound << std::endl;
    cv::Mat image(yBound, xBound, CV_8UC3, cv::Scalar(0, 0, 0));
    std::cout << "image size: " << image.size() << std::endl;
    // East is x, North is y
    for (const auto& point : points) {
        double x = (point.x - xMinMax.first) * coef;
        // invert y to have the same orientation as the GPS
        double y = yBound - (point.y - yMinMax.first) * coef;
        // std::cout << "x: " << x << " y: " << y << std::endl;
        cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
    }
    cv::imshow("Points", image);
    cv::waitKey(0);
}

int main() {
    std::vector<Point> points = parseCSV("../bfp_gps.csv");
    drawPoints(points);

    return 0;
}