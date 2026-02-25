#pragma once

#include <cstdint>
#include <string>

namespace robot_base
{

struct CanFrame
{
  uint32_t id{0};
  uint8_t dlc{0};
  uint8_t data[8]{};
};

// TODO(you): Replace these stubs with real SocketCAN IO (read/write).
class CanIo
{
public:
  explicit CanIo(std::string ifname) : ifname_(std::move(ifname)) {}
  bool open();
  bool read(CanFrame & out_frame);
  bool write(const CanFrame & frame);
  const std::string & ifname() const { return ifname_; }

private:
  std::string ifname_;
  bool opened_{false};
};

}  // namespace robot_base
