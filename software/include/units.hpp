#ifndef PSM_UNITS_HPP
#define PSM_UNITS_HPP
#include "fixpoint.hpp"
#include <span>

namespace psm {

template <uint8_t... Label> class UnitBase {
  static constexpr uint8_t label_[]{Label...};

  signed_fixed<16, 16> value_{};

public:
  using value_type = signed_fixed<16, 16>;

  constexpr UnitBase() = default;
  constexpr UnitBase(const UnitBase &) = default;
  constexpr UnitBase(UnitBase &&) = default;
  constexpr UnitBase(value_type value) : value_(value) {}

  constexpr UnitBase &operator=(const UnitBase &) = default;
  constexpr UnitBase &operator=(UnitBase &&) = default;
  constexpr UnitBase &operator=(value_type value) noexcept {
    value_ = value;
    return *this;
  }

  constexpr value_type value() const noexcept { return value_; }

  constexpr std::span<uint8_t> label() const noexcept {
    return {label_, sizeof...(Label)};
  }

  static_assert(sizeof(value_type) <= 4);
};

class Voltage : public UnitBase<'V'> {
public:
  using Base = UnitBase<'V'>;
  using Base::Base;
  using Base::operator=;
};

class Current : public UnitBase<'A'> {
public:
  using Base = UnitBase<'A'>;
  using Base::Base;
  using Base::operator=;
};

class Power : public UnitBase<'W'> {
public:
  using Base = UnitBase<'W'>;
  using Base::Base;
  using Base::operator=;
};

class Resistance : public UnitBase<'O', 'h', 'm'> {
public:
  using Base = UnitBase<'O', 'h', 'm'>;
  using Base::Base;
  using Base::operator=;
};

class Celcius : public UnitBase<u'°', 'C'> {
public:
  using Base = UnitBase<u'°', 'C'>;
  using Base::Base;
  using Base::operator=;
};

} // namespace psm

#endif
