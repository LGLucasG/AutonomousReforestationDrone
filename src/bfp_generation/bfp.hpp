#include <opencv2/opencv.hpp>
#include <iostream>

struct GPSPoint
{
    double latitude;
    double longitude;
};

int check_points(std::vector<cv::Point> points, int min_points = 3, int max_points = 10);
std::vector<cv::Point> find_contour(cv::Mat image);
std::vector<cv::Point> make_unique(std::vector<cv::Point> points);
std::vector<std::vector<cv::Point>> draw_lines(std::vector<cv::Point> points, cv::Mat result);
std::vector<cv::Point> clockwise_order(std::vector<cv::Point> points);
int find_largest_line(std::vector<std::vector<cv::Point>> lines);
std::vector<cv::Point2f> create_ortho_to(std::vector<std::vector<cv::Point>> lines, std::vector<cv::Point> points, int index, float pos);
std::vector<cv::Point2f> find_best_ortho(std::vector<std::vector<cv::Point>> lines, std::vector<cv::Point> points, int index, int iter);
std::vector<cv::Point> bfp_disordered(std::vector<std::vector<cv::Point>> lines, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f p4, int density);
std::vector<cv::Point> remove_point_if_outside_polygon(std::vector<cv::Point> poly, std::vector<cv::Point> points);
std::vector<cv::Point> bfp_ordered(std::vector<cv::Point> bfp_d);
void create_random_poly(time_t seed, std::string filename, bool display = 0);
std::vector<cv::Point> compute_bfp(std::string imagename, int density = 10, int spacingInterval = 10, bool display = 0);
void choosePoints(int event, int x, int y, int flags, void *param);
void create_mask(std::string image_name);
std::vector<cv::Point> get_bfp_points(cv::Mat image, std::vector<cv::Point> bfp, int spacingInterval, bool display = 0);
std::vector<cv::Point> get_image_points(std::string image_name);
std::vector<GPSPoint> get_gps_points(const std::string &filename);
std::vector<std::array<double, 2>> transformToGlobal(const std::vector<cv::Point> &imagePoints,
                                                    const cv::Point &p1, const cv::Point &p2,
                                                    const std::array<double, 2> &global1, const std::array<double, 2> &global2);
std::vector<GPSPoint> transformToGPS(const std::vector<cv::Point> &imagePoints,
                                     const cv::Point &p1, const cv::Point &p2,
                                     const GPSPoint &gps1, const GPSPoint &gps2);
