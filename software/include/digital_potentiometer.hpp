#ifndef PSM_DIGITAL_POTENTIOMETER_HPP
#define PSM_DIGITAL_POTENTIOMETER_HPP
#include "error.hpp"

#include <cstdint>

namespace psm {

class DigitalPotentiometer {
public:
  constexpr Error enable();

  constexpr Error disable();

  constexpr Error wiper(std::uint32_t code);

  constexpr tl::expected<Error, std::uint32_t> wiper();

  constexpr Error increment();

  constexpr Error decrement();

private:
};

} // namespace psm

#endif
