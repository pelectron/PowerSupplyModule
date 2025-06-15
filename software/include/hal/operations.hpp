#ifndef HAL_OPERATIONS_HPP
#define HAL_OPERATIONS_HPP

#include "poly.hpp"
namespace hal {
POLY_METHOD(enable);
POLY_METHOD(disable);
POLY_METHOD(start);
POLY_METHOD(trigger);
POLY_METHOD(stop);
POLY_METHOD(sample)
POLY_METHOD(write);
POLY_METHOD(read);
POLY_METHOD(transceive);
POLY_METHOD(register_callback);
POLY_METHOD(init);
POLY_METHOD(deinit);
POLY_METHOD(configure);
POLY_METHOD(create);
} // namespace hal
#endif
