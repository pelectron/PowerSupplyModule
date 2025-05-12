#ifndef PSM_CONVERTER_HPP
#define PSM_CONVERTER_HPP

#include "poly.hpp"
#include "poly/property.hpp"
#include "units.hpp"

namespace psm {

namespace conv {

POLY_PROPERTY(voltage);
POLY_PROPERTY(current);
POLY_PROPERTY(enabled);
POLY_PROPERTY(temperature);
POLY_PROPERTY(specs);

struct Specs {
  Voltage v_min;
  Voltage v_max;
  Voltage v_granularity;
  Current i_min;
  Current i_max;
  Current i_granularity;
};

template <poly::Storage S>
using Handle = poly::Struct<
    S,
    poly::type_list<voltage(Voltage), current(Current), enabled(bool),
                    const temperature(const Celcius),
                    const specs(const Specs &)>,
    poly::type_list<>>;

template <poly::Storage S>
class BasicConverter
    : public poly::Struct<
          S,
          poly::type_list<voltage(Voltage), current(Current), enabled(bool),
                          const temperature(const Celcius),
                          const specs(const Specs &)>,
          poly::type_list<>> {};

using ConverterReference = BasicConverter<poly::ref_storage>;

} // namespace conv
} // namespace psm

#endif
