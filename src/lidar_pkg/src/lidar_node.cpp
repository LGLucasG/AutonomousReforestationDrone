#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "lidar_pkg/lidarlite_v3.h"

using namespace std::chrono_literals;

class LidarPublisher : public rclcpp::Node {
public:
    LidarPublisher() : Node("lidar_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("lidar_distance", 10);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&LidarPublisher::timer_callback, this));
        lidar_.i2c_init();
	lidar_.i2c_connect(0x62);  // Initialize LIDAR device
        lidar_.configure(0);  // Configure LIDAR settings
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::Float64();
	__u8 busyFlag = lidar_.getBusyFlag();
	int distance;
	if (busyFlag == 0x00)
        {
		lidar_.takeRange();
        	distance = lidar_.readDistance();  // Get distance from LIDAR
	}
        message.data = static_cast<double>(distance);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    size_t count_;
    LIDARLite_v3 lidar_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPublisher>());
    rclcpp::shutdown();
    return 0;
}
