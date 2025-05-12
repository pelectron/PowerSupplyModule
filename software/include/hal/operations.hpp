#ifndef HAL_OPERATIONS_HPP
#define HAL_OPERATIONS_HPP

#include "poly.hpp"
namespace hal {
POLY_METHOD(write);
POLY_METHOD(read);
POLY_METHOD(register_callback);
POLY_METHOD(init);
POLY_METHOD(configure);
POLY_METHOD(create);
} // namespace hal
#endif
