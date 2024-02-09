#include <opencv2/opencv.hpp>
#include <iostream>
#include "bfp.hpp"
#include <fstream>

// def check_polyPoints
int check_points(std::vector<cv::Point> points, int min_points, int max_points)
{
    // check points
    if (points.size() < min_points or points.size() > max_points)
    {
        std::cout << "Bad approximation, not enough/too many points" << std::endl;
        return 0;
    }
    return 1;
}

// def find_contour
std::vector<cv::Point> find_contour(cv::Mat image)
{

    // Convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Apply edge detection
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // cut contour into straight lines
    std::vector<cv::Point> points;
    for (int i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], points, cv::arcLength(contours[i], true) * 0.01, true);
    }

    if (!check_points(points, 4))
    {
        // empty points
        points.clear();
        // extract points of contours
        for (int i = 0; i < contours.size(); i++)
        {
            for (int j = 0; j < contours[i].size(); j++)
            {
                points.push_back(contours[i][j]);
            }
        }
    }

    if (!check_points(points))
    // retry but with 3 points minimum
    {
        // empty points
        points.clear();
        for (int i = 0; i < contours.size(); i++)
        {
            cv::approxPolyDP(contours[i], points, cv::arcLength(contours[i], true) * 0.01, true);
        }
    }
    if (!check_points(points))
    {
        std::cout << "All checks failed, impossible to find a correct polygon." << std::endl;
        points = {};
    }

    return points;
}

// def make points unique
std::vector<cv::Point> make_unique(std::vector<cv::Point> points)
{
    // make each point of unique
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = i + 1; j < points.size(); j++)
        {
            if (points.size() > 4 and (points[i] == points[j] or norm(points[i] - points[j]) < 100))
            {
                points.erase(points.begin() + j);
                j--;
            }
        }
    }
    return points;
}

// def draw_lines
std::vector<std::vector<cv::Point>> draw_lines(std::vector<cv::Point> points, cv::Mat result)
{
    // draw the lines
    std::vector<std::vector<cv::Point>> lines;
    for (int i = 0; i < points.size(); i++)
    {
        std::vector<cv::Point> line;
        line.push_back(points[i]);
        line.push_back(points[(i + 1) % points.size()]);
        lines.push_back(line);
    }

    // display the lines with different colors
    srand(time(NULL));
    for (int i = 0; i < lines.size(); i++)
    {
        cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
        cv::drawContours(result, lines, i, color, 2);
    }
    return lines;
}

// def clockwise_order
std::vector<cv::Point> clockwise_order(std::vector<cv::Point> points)
{
    // reorder points in clockwise order (referencial = center of the poly)
    cv::Point center = cv::Point(0, 0);
    for (int i = 0; i < points.size(); i++)
    {
        center.x += points[i].x;
        center.y += points[i].y;
    }
    center.x /= points.size();
    center.y /= points.size();

    // sort points in clockxwise order with center as center
    std::sort(points.begin(), points.end(), [center](cv::Point a, cv::Point b)
              { return atan2(a.y - center.y, a.x - center.x) < atan2(b.y - center.y, b.x - center.x); });
    return points;
}

// find largest line
int find_largest_line(std::vector<std::vector<cv::Point>> lines)
{
    // find the largest line and display it in red
    int max = 0;
    int index = 0;
    for (int i = 0; i < lines.size(); i++)
    {
        if (cv::norm(lines[i][0] - lines[i][1]) > max)
        {
            max = cv::norm(lines[i][0] - lines[i][1]);
            index = i;
        }
    }
    return index;
}

// def create ortho to
std::vector<cv::Point2f> create_ortho_to(std::vector<std::vector<cv::Point>> lines, std::vector<cv::Point> points, int index, float pos)
{
    cv::Point2f p1 = lines[index][0];
    cv::Point2f p2 = lines[index][1];
    cv::Point2f p3, p4;
    p3.x = p1.x + (p2.x - p1.x) * pos;
    p3.y = p1.y + (p2.y - p1.y) * pos;
    p4.x = p3.x + (p2.y - p1.y);
    p4.y = p3.y - (p2.x - p1.x);
    // if p4 is outside the polygon, make it closer
    int i = 0;
    while (cv::pointPolygonTest(points, p4, false) < 0)
    {
        p4.x = p3.x + (p4.x - p3.x) * -0.98;
        p4.y = p3.y + (p4.y - p3.y) * -0.98;
        i++;
        if (i > 1000)
        {
            break;
        }
    }
    return {p3, p4};
}

// def find best ortho
std::vector<cv::Point2f> find_best_ortho(std::vector<std::vector<cv::Point>> lines, std::vector<cv::Point> points, int index, int iter)
{
    std::vector<cv::Point2f> best_ortho;
    float max = 0;
    for (int i = 0; i < iter; i++)
    {
        float pos = 0.1f + i * 0.8f / iter;
        std::vector<cv::Point2f> ortho = create_ortho_to(lines, points, index, pos);
        float length = cv::norm(ortho[0] - ortho[1]);
        if (length > max)
        {
            max = length;
            best_ortho = ortho;
        }
    }
    return best_ortho;
}

// def bfp_chaotic
std::vector<cv::Point> bfp_disordered(std::vector<std::vector<cv::Point>> lines, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4, int density)
{
    // now draw 10 times the red line along the green one but orthogonally. The green line should be the mediator of the red lines
    std::vector<cv::Point> bfp;
    cv::Point2f offset = (p4 - p3) / (density * 2);
    for (int i = 0; i < density + 1; i++)
    {
        cv::Point2f p5, p6;
        p5.x = p2.x + offset.x + i * (p4.x - p3.x) / density;
        p5.y = p2.y + offset.y + i * (p4.y - p3.y) / density;
        // p6 is the mirror of p5 along the green line
        p6.x = p1.x + offset.x + i * (p4.x - p3.x) / density;
        p6.y = p1.y + offset.y + i * (p4.y - p3.y) / density;

        // find intersection between p5p6 and the all the lines of the polygon
        for (int j = 0; j < lines.size(); j++)
        {
            cv::Point2f p7 = lines[j][0];
            cv::Point2f p8 = lines[j][1];
            cv::Point2f p9;
            p9.x = ((p5.x * p6.y - p5.y * p6.x) * (p7.x - p8.x) - (p5.x - p6.x) * (p7.x * p8.y - p7.y * p8.x)) / ((p5.x - p6.x) * (p7.y - p8.y) - (p5.y - p6.y) * (p7.x - p8.x));
            p9.y = ((p5.x * p6.y - p5.y * p6.x) * (p7.y - p8.y) - (p5.y - p6.y) * (p7.x * p8.y - p7.y * p8.x)) / ((p5.x - p6.x) * (p7.y - p8.y) - (p5.y - p6.y) * (p7.x - p8.x));
            // if p9 is on the line, draw it
            if (cv::norm(p9 - p7) + cv::norm(p9 - p8) - cv::norm(p7 - p8) < 0.1)
            {
                bfp.push_back(p9);
            }
        }
    }
    return bfp;
}

// def remove_point_if_outside_polygon
std::vector<cv::Point> remove_point_if_outside_polygon(std::vector<cv::Point> poly, std::vector<cv::Point> points)
{
    // remove points if they are outside the polygon
    for (int i = 0; i < points.size(); i++)
    {
        if (cv::pointPolygonTest(poly, points[i], false) < -5)
        {
            points.erase(points.begin() + i);
            i--;
        }
    }
    return points;
}

// def bfp_ordered
std::vector<cv::Point> bfp_ordered(std::vector<cv::Point> bfp_d)
{
    // order the points
    std::vector<cv::Point> bfp_o;
    for (int i = 0; i < bfp_d.size(); i += 2)
    {
        if ((i / 2) % 2 == 0)
        {
            bfp_o.push_back(bfp_d[i]);
            bfp_o.push_back(bfp_d[i + 1]);
        }
        else
        {
            bfp_o.push_back(bfp_d[i + 1]);
            bfp_o.push_back(bfp_d[i]);
        }
    }
    return bfp_o;
}

void create_random_poly(time_t seed, std::string filename, bool display)
{
    srand(seed);
    std::cout << "Seed: " << seed << std::endl;
    // Create a black image
    cv::Mat image(500, 500, CV_8UC1, cv::Scalar(0));

    // Generate random points for the polygon
    std::vector<cv::Point> points;
    while (true)
    {
        std::vector<cv::Point> pts;
        for (int i = 0; i < 4; i++)
        {
            int x = rand() % 500;
            int y = rand() % 500;
            pts.push_back(cv::Point(x, y));
        }
        try
        {
            if (cv::isContourConvex(pts))
            {
                points = pts;
                break;
            }
        }
        catch (const std::exception &e)
        {
            std::cout << "Not convex polygon" << std::endl;
            std::cerr << e.what() << '\n';
        }
    }
    // Create a white polygon on the black image
    cv::fillConvexPoly(image, points.data(), points.size(), cv::Scalar(255));

    // save the image
    //  if polygon.png exists, save as polygon1.png, etc.
    cv::imwrite("../polygon1.png", image);

    // display the image
    if (display)
    {
        cv::imshow("image", image);
        cv::waitKey(0);
    }
    return;
}

std::vector<cv::Point> get_bfp_points(cv::Mat image, std::vector<cv::Point> bfp, int spacingInterval, bool display)
{
    std::vector<cv::Point> bfp_points;
    // for all the points of bfp, compute the line between two points (i and i+1)
    // then, compute the points of the line with a spacing of 10 pixels
    for (int i = 0; i < bfp.size() - 1; i++)
    {
        cv::Point p1 = bfp[i];
        cv::Point p2 = bfp[i + 1];
        cv::Point p3;
        // compute the line between p1 and p2
        cv::Vec4f line;
        cv::fitLine(cv::Mat(std::vector<cv::Point>{p1, p2}), line, cv::DIST_L2, 0, 0.01, 0.01);
        // display the line
        cv::line(image, p1, p2, cv::Scalar(255, 0, 0), 2);
        cv::LineIterator itLine(cv::Mat::zeros(image.size(), image.type()), p1, p2, 8);
        for (int i = 0; i < itLine.count; i++, ++itLine)
        {
            if (i % spacingInterval == 0)
            {
                bfp_points.push_back(itLine.pos());
            }
        }
    }
    if (display)
    {
        for (int i = 0; i < bfp_points.size(); i++)
        {
            cv::circle(image, bfp_points[i], 2, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("image", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    return bfp_points;
}

std::vector<cv::Point> compute_bfp(std::string imagename, int density, int spacingInterval, bool display)
{
    // Read the image
    cv::Mat image = cv::imread(imagename);

    // create result image for display
    cv::Mat result = cv::Mat::zeros(image.size(), CV_8UC3);

    // Find contours
    std::vector<cv::Point> polyPoints = find_contour(image);
    if (polyPoints.empty())
        return {};
    if (!check_points(polyPoints))
        return {};

    // remove duplicate points
    polyPoints = make_unique(polyPoints);
    if (!check_points(polyPoints))
        return {};

    // reorder points in clockwise order
    polyPoints = clockwise_order(polyPoints);

    // draw the lines
    std::vector<std::vector<cv::Point>> lines = draw_lines(polyPoints, result);

    // find the largest line and display it in red
    int index = find_largest_line(lines);
    cv::drawContours(result, lines, index, cv::Scalar(0, 0, 255), 2);

    // find the orthogonal line to the largest line and display it in green
    // std::vector<cv::Point2f> ortho = create_ortho_to(lines, polyPoints, index, 0.5);
    std::vector<cv::Point2f> ortho = find_best_ortho(lines, polyPoints, index, 100);
    cv::Point2f p1 = lines[index][0];
    cv::Point2f p2 = lines[index][1];
    cv::Point2f p3 = ortho[0];
    cv::Point2f p4 = ortho[1];
    cv::line(result, p3, p4, cv::Scalar(0, 255, 0), 2);

    // define a vector of line
    std::vector<cv::Point> bfp_d, bfp_o;
    bfp_d = bfp_disordered(lines, p1, p2, p3, p4, density);

    bfp_d = remove_point_if_outside_polygon(polyPoints, bfp_d);

    bfp_o = bfp_ordered(bfp_d);

    // Display results
    if (display)
    {
        // draw the points of bfp
        for (int i = 0; i < bfp_d.size(); i++)
        {
            cv::circle(result, bfp_d[i], 2, cv::Scalar(i * 255 / bfp_d.size(), 0, (bfp_d.size() - i) * 255 / bfp_d.size()), 2);
        }

        // draw the lines of bfp
        for (int i = 1; i < bfp_o.size(); i++)
        {
            cv::line(result, bfp_o[i - 1], bfp_o[i], cv::Scalar(i * 255 / bfp_o.size(), 0, (bfp_o.size() - i) * 255 / bfp_o.size()), 2);
        }

        cv::imshow("Result", result);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    std::vector<cv::Point> bfp_points;
    bfp_points = get_bfp_points(image, bfp_o, spacingInterval, display);
    return bfp_points;
}

void choosePoints(int event, int x, int y, int flags, void *param)
{
    std::vector<cv::Point> *points = static_cast<std::vector<cv::Point> *>(param);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        points->push_back(cv::Point(x, y));
        cv::circle(*points, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
        std::cout << "Point : " << x << ", " << y << std::endl;
    }
}

void create_mask(std::string image_name)
{
    std::vector<cv::Point> points;
    cv::Mat image = cv::imread(image_name);
    if (image.empty())
    {
        std::cerr << "Error: Unable to open image" << std::endl;
        return;
    }
    cv::imshow("image", image);
    cv::setMouseCallback("image", choosePoints, &points);
    cv::imshow("image", image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    // create mask
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(points);
    cv::fillPoly(mask, contours, cv::Scalar(255));
    cv::imshow("mask", mask);
    cv::waitKey(0);
    cv::destroyAllWindows();
    cv::imwrite("../mask.png", mask);
    // std::cout << "Mask created" << std::endl;
    return;
}

std::vector<cv::Point> get_image_points(std::string image_name)
{
    // ask for img points
    std::vector<cv::Point> imgPoints;
    cv::Mat image = cv::imread(image_name);
    if (image.empty())
    {
        std::cerr << "Error: Unable to open image" << std::endl;
        return {};
    }
    cv::imshow("image", image);
    cv::setMouseCallback("image", choosePoints, &imgPoints);
    cv::imshow("image", image);

    std::cout << "Please select 2 points on the image" << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    return imgPoints;
}

std::vector<GPSPoint> get_gps_points(const std::string &filename)
{
    // if gps_keypoints.csv exists, take the points from it
    std::ifstream file(filename);
    std::vector<GPSPoint> gpsPoints;
    if (file.is_open())
    {
        std::cout << "gps_keypoints.csv file found." << std::endl;
        std::string line;
        std::getline(file, line); // skip first line
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string data;
            GPSPoint gpsPoint;
            std::getline(ss, data, ',');
            gpsPoint.latitude = std::stod(data);
            std::getline(ss, data, ',');
            gpsPoint.longitude = std::stod(data);
            gpsPoints.push_back(gpsPoint);
        }
    }
    else
    {
        std::cout << "No gps_keypoints.csv file found." << std::endl;
        std::cout << "Enter the GPS coordinates of the first point : " << std::endl;
        GPSPoint gpsPoint;
        std::cout << "latitude : ";
        std::cin >> gpsPoint.latitude;
        std::cout << "longitude : ";
        std::cin >> gpsPoint.longitude;
        gpsPoints.push_back(gpsPoint);
        std::cout << "Enter the GPS coordinates of the second point : " << std::endl;
        std::cout << "latitude : ";
        std::cin >> gpsPoint.latitude;
        std::cout << "longitude : ";
        std::cin >> gpsPoint.longitude;
        gpsPoints.push_back(gpsPoint);
    }
    return gpsPoints;
}

// linear interpolation of all points between p1 and p2
std::vector<std::array<double,2>> transformToGlobal(const std::vector<cv::Point> &imagePoints,
                                     const cv::Point &p1, const cv::Point &p2,
                                     const std::array<double, 2> &global1, const std::array<double, 2> &global2)
{
    std::vector<std::array<double, 2>> globalPoints;

    // Calculer les échelles de transformation
    double scaleX = (global2[0] - global1[0]) / (p2.x - p1.x);
    double scaleY = (global2[1] - global1[1]) / (p2.y - p1.y);

    // Calculer les décalages
    double offsetX = global1[0] - scaleX * p1.x;
    double offsetY = global1[1] - scaleY * p1.y;

    // Transformer chaque point
    for (const auto &point : imagePoints)
    {
        std::array<double, 2> globalPoint;
        globalPoint[0] = scaleX * point.x + offsetX;
        globalPoint[1] = scaleY * point.y + offsetY;
        globalPoints.push_back(globalPoint);
    }

    return globalPoints;
}

std::vector<GPSPoint> transformToGPS(const std::vector<cv::Point> &imagePoints,
                                     const cv::Point &p1, const cv::Point &p2,
                                     const GPSPoint &gps1, const GPSPoint &gps2)
{
    std::vector<GPSPoint> gpsPoints;

    // Calculer les échelles de transformation
    double scaleX = (gps2.longitude - gps1.longitude) / (p2.x - p1.x);
    double scaleY = (gps2.latitude - gps1.latitude) / (p2.y - p1.y);

    // Calculer les décalages
    double offsetX = gps1.longitude - scaleX * p1.x;
    double offsetY = gps1.latitude - scaleY * p1.y;

    // Transformer chaque point
    for (const auto &point : imagePoints)
    {
        GPSPoint gpsPoint;
        gpsPoint.longitude = scaleX * point.x + offsetX;
        gpsPoint.latitude = scaleY * point.y + offsetY;
        gpsPoints.push_back(gpsPoint);
    }

    return gpsPoints;
}