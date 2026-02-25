#include "robot_base/can_bridge.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_base/msg/encoder_ticks.hpp"
#include <algorithm>

using namespace std::chrono_literals;

namespace robot_base
{

bool CanIo::open()
{
  // TODO: implement SocketCAN open() (socket(), bind(), etc.)
  opened_ = true;
  return opened_;
}

bool CanIo::read(CanFrame & out_frame)
{
  // TODO: implement SocketCAN read() (recvfrom/recvmsg)
  // Return false if no frame received.
  (void)out_frame;
  return false;
}

bool CanIo::write(const CanFrame & frame)
{
  // TODO: implement SocketCAN write() (sendto)
  (void)frame;
  return opened_;
}

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode() : Node("can_bridge")
  {
    // Parameters
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    // Simulate mode: generates fake encoder ticks so you can test the pipeline without hardware.
    simulate_ = declare_parameter<bool>("simulate", true);
    sim_left_ticks_per_s_ = declare_parameter<double>("simulate_left_ticks_per_s", 500.0);
    sim_right_ticks_per_s_ = declare_parameter<double>("simulate_right_ticks_per_s", 500.0);

    pub_encoders_ = create_publisher<robot_base::msg::EncoderTicks>("/encoders_raw", rclcpp::QoS(10));

    // Initialize CAN IO (stub for now)
    can_io_ = std::make_unique<CanIo>(can_interface_);
    if (!simulate_) {
      if (!can_io_->open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CAN interface: %s", can_interface_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "CAN interface opened: %s", can_interface_.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(), "SIMULATE mode enabled: publishing fake encoder ticks.");
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CanBridgeNode::onTimer, this));
  }

private:
  void onTimer()
  {
    robot_base::msg::EncoderTicks msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";  // semantic only; not used for TF

    if (simulate_) {
      // Simple integrator in ticks
      const double dt = 1.0 / std::max(1.0, publish_rate_hz_);
      left_ticks_ += static_cast<int64_t>(sim_left_ticks_per_s_ * dt);
      right_ticks_ += static_cast<int64_t>(sim_right_ticks_per_s_ * dt);
      msg.left_ticks = left_ticks_;
      msg.right_ticks = right_ticks_;
      pub_encoders_->publish(msg);
      return;
    }

    // TODO: Replace with real parsing logic for your STM32 CAN frames.
    // Example flow:
    // 1) read frames in a loop (or non-blocking) until you have left/right ticks
    // 2) fill msg.left_ticks / msg.right_ticks
    // 3) publish

    CanFrame frame;
    if (can_io_->read(frame)) {
      // TODO: parse frame -> ticks
      (void)frame;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No CAN frames received (stub). Implement CanIo::read() and parsing.");
    }
  }

  std::string can_interface_;
  double publish_rate_hz_{50.0};

  bool simulate_{true};
  double sim_left_ticks_per_s_{500.0};
  double sim_right_ticks_per_s_{500.0};

  int64_t left_ticks_{0};
  int64_t right_ticks_{0};

  rclcpp::Publisher<robot_base::msg::EncoderTicks>::SharedPtr pub_encoders_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<CanIo> can_io_;
};

}  // namespace robot_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_base::CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
