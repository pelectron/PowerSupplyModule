#ifndef IRQ_HPP
#define IRQ_HPP
#include "clocks.hpp"

namespace hal {
namespace irq {

enum class Error {

};

Error enable(clock::PrimaryClock clock);
Error enable(clock::SecondaryClock clock);

Error disable(clock::PrimaryClock clock);
Error disable(clock::SecondaryClock clock);
} // namespace irq
} // namespace hal
#endif
