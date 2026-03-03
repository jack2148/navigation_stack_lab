
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <vector>

class CanDiffDriveHW : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override
  {
    pos_.assign(2, 0.0);
    vel_.assign(2, 0.0);
    cmd_.assign(2, 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    return {
      {"left_wheel", hardware_interface::HW_IF_POSITION, &pos_[0]},
      {"right_wheel", hardware_interface::HW_IF_POSITION, &pos_[1]},
      {"left_wheel", hardware_interface::HW_IF_VELOCITY, &vel_[0]},
      {"right_wheel", hardware_interface::HW_IF_VELOCITY, &vel_[1]}
    };
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    return {
      {"left_wheel", hardware_interface::HW_IF_VELOCITY, &cmd_[0]},
      {"right_wheel", hardware_interface::HW_IF_VELOCITY, &cmd_[1]}
    };
  }

  hardware_interface::return_type read() override
  {
    // TODO: Replace with real CAN read (0x101)
    pos_[0] += vel_[0] * 0.02;
    pos_[1] += vel_[1] * 0.02;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write() override
  {
    // TODO: Convert cmd_ (rad/s) to RPM and send via CAN 0x100
    vel_[0] = cmd_[0];
    vel_[1] = cmd_[1];
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> cmd_;
};

PLUGINLIB_EXPORT_CLASS(CanDiffDriveHW, hardware_interface::SystemInterface)
