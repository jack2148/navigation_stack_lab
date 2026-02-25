#pragma once

#include <cstdint>
#include <cmath>

namespace robot_base
{

inline double ticksToRadians(const int64_t delta_ticks, const double ticks_per_rev)
{
  // rad = ticks / (ticks/rev) * 2*pi
  return (static_cast<double>(delta_ticks) / ticks_per_rev) * 2.0 * M_PI;
}

}  // namespace robot_base
