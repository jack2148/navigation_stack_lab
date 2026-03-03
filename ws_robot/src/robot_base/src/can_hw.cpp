#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

class CanDiffDriveHW : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // ---- Defaults (override via URDF <ros2_control><hardware><param ...>) ----
    left_joint_name_  = "left_wheel_joint";
    right_joint_name_ = "right_wheel_joint";

    can_iface_ = "can0";
    can_id_rx_ticks_ = 0x101;
    can_id_tx_rpm_   = 0x100;

    // Your measured ticks_per_wheel_rev (wheel 1 rev)
    ticks_per_wheel_rev_ = 440.5;

    // IMPORTANT:
    // STM32 already applies CMD_SIGN internally, so Jetson should usually keep +1/+1 here.
    cmd_sign_l_ = +1.0;
    cmd_sign_r_ = +1.0;

    // Encoder sign (if your ticks decrease on forward, set -1)
    enc_sign_l_ = +1.0;
    enc_sign_r_ = +1.0;

    // If STM32 expects motor RPM directly (rare in your posted STM32 code), enable true
    send_motor_rpm_ = false;
    gear_ratio_ = 15.0;
    max_motor_rpm_ = 30000.0;

    // ---- Parameter loader (from URDF ros2_control hardware params) ----
    auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
      const auto it = info_.hardware_parameters.find(key);
      return (it == info_.hardware_parameters.end()) ? def : it->second;
    };

    try {
      left_joint_name_  = get_param("left_joint_name", left_joint_name_);
      right_joint_name_ = get_param("right_joint_name", right_joint_name_);

      can_iface_ = get_param("can_iface", can_iface_);

      // allow "0x101" style
      can_id_rx_ticks_ = static_cast<uint32_t>(std::stoul(get_param("can_id_rx_ticks", "0x101"), nullptr, 0));
      can_id_tx_rpm_   = static_cast<uint32_t>(std::stoul(get_param("can_id_tx_rpm", "0x100"),   nullptr, 0));

      ticks_per_wheel_rev_ = std::stod(get_param("ticks_per_wheel_rev", std::to_string(ticks_per_wheel_rev_)));

      // New key: cmd_sign_l/r
      // Legacy key: wheel_sign_l/r (in case you already wrote URDF with these)
      cmd_sign_l_ = std::stod(get_param("cmd_sign_l", get_param("wheel_sign_l", "+1")));
      cmd_sign_r_ = std::stod(get_param("cmd_sign_r", get_param("wheel_sign_r", "+1")));

      enc_sign_l_ = std::stod(get_param("enc_sign_l", "+1"));
      enc_sign_r_ = std::stod(get_param("enc_sign_r", "+1"));

      const std::string sm = get_param("send_motor_rpm", "false");
      send_motor_rpm_ = (sm == "true" || sm == "1");

      gear_ratio_ = std::stod(get_param("gear_ratio", std::to_string(gear_ratio_)));
      max_motor_rpm_ = std::stod(get_param("max_motor_rpm", std::to_string(max_motor_rpm_)));
    } catch (...) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (ticks_per_wheel_rev_ <= 0.0) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Allocate state/command storage
    pos_.assign(2, 0.0);
    vel_.assign(2, 0.0);
    cmd_.assign(2, 0.0);

    last_ticks_valid_ = false;
    last_ticks_l_ = 0;
    last_ticks_r_ = 0;

    if (!open_can_socket_()) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (can_fd_ < 0) {
      if (!open_can_socket_()) {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    last_ticks_valid_ = false;
    vel_[0] = vel_[1] = 0.0;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (can_fd_ >= 0) {
      ::close(can_fd_);
      can_fd_ = -1;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(left_joint_name_,  hardware_interface::HW_IF_POSITION, &pos_[0]);
    state_interfaces.emplace_back(right_joint_name_, hardware_interface::HW_IF_POSITION, &pos_[1]);
    state_interfaces.emplace_back(left_joint_name_,  hardware_interface::HW_IF_VELOCITY, &vel_[0]);
    state_interfaces.emplace_back(right_joint_name_, hardware_interface::HW_IF_VELOCITY, &vel_[1]);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(left_joint_name_,  hardware_interface::HW_IF_VELOCITY, &cmd_[0]);
    command_interfaces.emplace_back(right_joint_name_, hardware_interface::HW_IF_VELOCITY, &cmd_[1]);
    return command_interfaces;
  }

  // ✅ Humble signature
  hardware_interface::return_type read(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & period) override
  {
    if (can_fd_ < 0) {
      return hardware_interface::return_type::ERROR;
    }

    struct can_frame frame{};
    bool got_ticks = false;
    int32_t ticks_l = 0;
    int32_t ticks_r = 0;

    // Drain socket (non-blocking)
    while (true) {
      const int n = ::read(can_fd_, &frame, sizeof(frame));
      if (n < 0) break; // EAGAIN => no more frames
      if (n != (int)sizeof(frame)) continue;

      const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
      if (can_id == can_id_rx_ticks_ && frame.can_dlc == 8) {
        ticks_l = le_i32_(frame.data + 0);
        ticks_r = le_i32_(frame.data + 4);
        got_ticks = true;
      }
    }

    const double dt = period.seconds();
    if (dt <= 0.0) {
      return hardware_interface::return_type::OK;
    }

    if (!got_ticks) {
      vel_[0] = 0.0;
      vel_[1] = 0.0;
      return hardware_interface::return_type::OK;
    }

    // ticks -> wheel angle(rad) (cumulative)
    const double ticks_l_s = enc_sign_l_ * static_cast<double>(ticks_l);
    const double ticks_r_s = enc_sign_r_ * static_cast<double>(ticks_r);

    const double angle_l = (ticks_l_s / ticks_per_wheel_rev_) * 2.0 * M_PI;
    const double angle_r = (ticks_r_s / ticks_per_wheel_rev_) * 2.0 * M_PI;

    if (!last_ticks_valid_) {
      pos_[0] = angle_l;
      pos_[1] = angle_r;
      vel_[0] = 0.0;
      vel_[1] = 0.0;
      last_ticks_l_ = ticks_l;
      last_ticks_r_ = ticks_r;
      last_ticks_valid_ = true;
      return hardware_interface::return_type::OK;
    }

    const double prev_l = pos_[0];
    const double prev_r = pos_[1];

    pos_[0] = angle_l;
    pos_[1] = angle_r;

    vel_[0] = (pos_[0] - prev_l) / dt;
    vel_[1] = (pos_[1] - prev_r) / dt;

    last_ticks_l_ = ticks_l;
    last_ticks_r_ = ticks_r;

    return hardware_interface::return_type::OK;
  }

  // ✅ Humble signature
  hardware_interface::return_type write(const rclcpp::Time & /*time*/,
                                        const rclcpp::Duration & /*period*/) override
  {
    if (can_fd_ < 0) {
      return hardware_interface::return_type::ERROR;
    }

    // rad/s -> wheel RPM
    double wheel_rpm_l = cmd_[0] * 60.0 / (2.0 * M_PI);
    double wheel_rpm_r = cmd_[1] * 60.0 / (2.0 * M_PI);

    // Optional sign mapping at Jetson side
    wheel_rpm_l *= cmd_sign_l_;
    wheel_rpm_r *= cmd_sign_r_;

    // Optional: if you ever need to send motor RPM directly
    if (send_motor_rpm_) {
      wheel_rpm_l *= gear_ratio_;
      wheel_rpm_r *= gear_ratio_;
      wheel_rpm_l = clamp_abs_(wheel_rpm_l, max_motor_rpm_);
      wheel_rpm_r = clamp_abs_(wheel_rpm_r, max_motor_rpm_);
    }

    const int16_t rpm_l_i16 = clamp_i16_(wheel_rpm_l);
    const int16_t rpm_r_i16 = clamp_i16_(wheel_rpm_r);

    struct can_frame frame{};
    frame.can_id = can_id_tx_rpm_;
    frame.can_dlc = 8;

    frame.data[0] = static_cast<uint8_t>(rpm_l_i16 & 0xFF);
    frame.data[1] = static_cast<uint8_t>((rpm_l_i16 >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(rpm_r_i16 & 0xFF);
    frame.data[3] = static_cast<uint8_t>((rpm_r_i16 >> 8) & 0xFF);

    frame.data[4] = frame.data[5] = frame.data[6] = frame.data[7] = 0;

    (void)::write(can_fd_, &frame, sizeof(frame));
    return hardware_interface::return_type::OK;
  }

private:
  bool open_can_socket_()
  {
    if (can_fd_ >= 0) {
      ::close(can_fd_);
      can_fd_ = -1;
    }

    can_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) return false;

    return true;
  }

  static int32_t le_i32_(const uint8_t *p)
  {
    return (int32_t)(
      (uint32_t)p[0] |
      ((uint32_t)p[1] << 8) |
      ((uint32_t)p[2] << 16) |
      ((uint32_t)p[3] << 24)
    );
  }

  static double clamp_abs_(double v, double abs_max)
  {
    if (v >  abs_max) return  abs_max;
    if (v < -abs_max) return -abs_max;
    return v;
  }

  static int16_t clamp_i16_(double v)
  {
    if (v >  32767.0) v =  32767.0;
    if (v < -32768.0) v = -32768.0;
    return static_cast<int16_t>(std::lround(v));
  }

private:
  std::string left_joint_name_;
  std::string right_joint_name_;

  std::string can_iface_;
  uint32_t can_id_rx_ticks_;
  uint32_t can_id_tx_rpm_;

  double ticks_per_wheel_rev_;
  double cmd_sign_l_;
  double cmd_sign_r_;
  double enc_sign_l_;
  double enc_sign_r_;

  bool send_motor_rpm_;
  double gear_ratio_;
  double max_motor_rpm_;

  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> cmd_;

  bool last_ticks_valid_{false};
  int32_t last_ticks_l_{0};
  int32_t last_ticks_r_{0};

  int can_fd_{-1};
};

PLUGINLIB_EXPORT_CLASS(CanDiffDriveHW, hardware_interface::SystemInterface)