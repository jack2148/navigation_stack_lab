#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>

using namespace std::chrono_literals;

namespace robot_sensors
{

class LidarStubNode : public rclcpp::Node
{
public:
  LidarStubNode() : Node("lidar_stub")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    scan_rate_hz_ = declare_parameter<double>("scan_rate_hz", 10.0);

    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max",  M_PI);
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 12.0);
    num_samples_ = declare_parameter<int>("num_samples", 360);

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, scan_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LidarStubNode::onTimer, this));

    RCLCPP_WARN(get_logger(), "LidarStubNode started. Publishing dummy /scan for pipeline testing.");
  }

private:
  void onTimer()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = frame_id_;

    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);

    const int n = std::max(1, num_samples_);
    scan.angle_increment = static_cast<float>((angle_max_ - angle_min_) / static_cast<double>(n));

    scan.time_increment = 0.0f; // unknown
    scan.scan_time = static_cast<float>(1.0 / std::max(1.0, scan_rate_hz_));

    scan.ranges.resize(n, static_cast<float>(range_max_));
    scan.intensities.resize(n, 0.0f);

    // Optional: create a fake "wall" in front (to see something in RViz)
    // For indices near 0 rad (front), set smaller ranges.
    for (int i = 0; i < n; ++i) {
      const double angle = angle_min_ + (angle_max_ - angle_min_) * (static_cast<double>(i) / (n - 1));
      if (std::abs(angle) < 0.15) {
        scan.ranges[i] = 2.0f;
      }
    }

    pub_scan_->publish(scan);
  }

  std::string frame_id_;
  std::string scan_topic_;
  double scan_rate_hz_{10.0};

  double angle_min_{-M_PI};
  double angle_max_{M_PI};
  double range_min_{0.05};
  double range_max_{12.0};
  int num_samples_{360};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sensors::LidarStubNode>());
  rclcpp::shutdown();
  return 0;
}
