#ifndef AD5293_HPP
#define AD5293_HPP

#include "hal/spi.hpp"
namespace psm {
class AD5293 {
public:
private:
  hal::spi::Master spi;
};
} // namespace psm
#endif
