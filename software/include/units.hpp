#ifndef PSM_UNITS_HPP
#define PSM_UNITS_HPP
#include "au/au.hh"
#include "error.hpp"
#include "fixpoint.hpp"
#include <cassert>
#include <cstdint>
#include <span>
#include <type_traits>
// TODO: add parsers and formatters for units

namespace std {
template <std::size_t I1, std::size_t I2, std::size_t F1, std::size_t F2>
struct common_type<signed_fixed<I1, F1>, signed_fixed<I2, F2>> {
  using type = std::conditional_t<
      I1 >= I2,
      std::conditional_t<F1 >= F2, signed_fixed<I1, F1>, signed_fixed<I1, F2>>,
      std::conditional_t<F1 >= F2, signed_fixed<I2, F1>, signed_fixed<I2, F2>>>;
};
template <std::size_t I, std::size_t F, typename T>
struct common_type<signed_fixed<I, F>, T> {
  using type = signed_fixed<I, F>;
};
template <std::size_t I, std::size_t F, typename T>
struct common_type<T, signed_fixed<I, F>> {
  using type = signed_fixed<I, F>;
};
template <std::size_t I1, std::size_t I2, std::size_t F1, std::size_t F2>
struct common_type<unsigned_fixed<I1, F1>, unsigned_fixed<I2, F2>> {
  using type =
      std::conditional_t<I1 >= I2,
                         std::conditional_t<F1 >= F2, unsigned_fixed<I1, F1>,
                                            unsigned_fixed<I1, F2>>,
                         std::conditional_t<F1 >= F2, unsigned_fixed<I2, F1>,
                                            unsigned_fixed<I2, F2>>>;
};
template <std::size_t I, std::size_t F, typename T>
struct common_type<unsigned_fixed<I, F>, T> {
  using type = signed_fixed<I, F>;
};
template <std::size_t I, std::size_t F, typename T>
struct common_type<T, unsigned_fixed<I, F>> {
  using type = signed_fixed<I, F>;
};

} // namespace std

template <class Unit, class Rep, std::size_t I, std::size_t F>
au::Quantity<Unit, Rep> operator*(au::Quantity<Unit, Rep> q,
                                  signed_fixed<I, F> s) {
  return au::QuantityMaker<Unit>{}(sign_extended_value(s) * q.in(Unit{}) >> F);
}

template <std::size_t I, std::size_t F, class Unit, class Rep>
au::Quantity<Unit, Rep> operator*(signed_fixed<I, F> s,
                                  au::Quantity<Unit, Rep> q) {
  return au::QuantityMaker<Unit>{}(sign_extended_value(s) * q.in(Unit{}) >> F);
}

template <class Unit, class Rep, std::size_t I, std::size_t F>
au::Quantity<Unit, Rep> operator/(au::Quantity<Unit, Rep> q,
                                  signed_fixed<I, F> s) {
  return au::QuantityMaker<Unit>{}(((q.in(Unit{}) << (I + F)) / s.value()) >>
                                   I);
}

struct Slewrate : decltype(au::Volts{} / au::micro(au::Seconds{})){};
using float_type = signed_fixed<12, 20>;
using Voltage = au::Quantity<au::Micro<au::Volts>, std::uint32_t>;
using Current = au::Quantity<au::Micro<au::Amperes>, std::uint32_t>;
using Frequency = au::Quantity<au::Hertz, std::uint32_t>;
using Resistance = au::Quantity<au::Milli<au::Ohms>, std::uint32_t>;
using Temperature = au::QuantityPoint<au::Centi<au::Celsius>, std::uint32_t>;
// feedback factor for the voltage feedback 1/8
inline constexpr signed_fixed<1, 4> voltage_feedback_factor{uint8_t{1u}};

template <class Derived, uint8_t... Label> class UnitBase {
public:
  using value_type = signed_fixed<12, 20>;
  using raw_value_type = typename value_type::raw_value_type;

  constexpr UnitBase() = default;
  constexpr UnitBase(const UnitBase &) = default;
  constexpr UnitBase(UnitBase &&) = default;
  constexpr explicit UnitBase(value_type value) : value_(value) {}

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

  constexpr auto operator<=>(const UnitBase &other) const {
    return value_ == other.value_  ? std::strong_ordering::equal
           : value_ < other.value_ ? std::strong_ordering::less
                                   : std::strong_ordering::greater;
  }

  friend constexpr Derived operator+(const Derived &a, const Derived &b) {
    return Derived{a.value_ + b.value_};
  }

  friend constexpr Derived operator-(const Derived &a, const Derived &b) {
    return Derived{a.value_ - b.value_};
  }

  friend constexpr Derived operator*(const Derived &a, const value_type &b) {
    return Derived{a.value_ * b};
  }

  friend constexpr Derived operator*(const value_type &a, const Derived &b) {
    return Derived{b.value_ * a};
  }

  template <std::integral T>
  friend constexpr Derived operator*(const Derived &a, const T &b) {
    if (b < 0) {
      return Derived{a * -value_type(raw_value_type(-b) << 22u)};
    } else {
      return Derived{a * value_type(raw_value_type(b << 22))};
    }
  }

  template <std::integral T>
  friend constexpr Derived operator*(const T &a, const Derived &b) {
    if (a < 0) {
      return Derived{b * -value_type(raw_value_type(-a) << 22)};
    } else {
      return Derived{b * value_type(raw_value_type(a << 22))};
    }
  }

  friend constexpr value_type operator/(const Derived &a, const Derived &b) {
    return value_type{resize<10, 22>(a.value_ / b.value_)};
  }

  friend constexpr Derived operator/(const Derived &a, const value_type &b) {
    return Derived{resize<10, 22>(a.value_ / b)};
  }

  template <std::integral T>
  friend constexpr Derived operator/(const Derived &a, const T &b) {
    if (b < 0) {
      return Derived{a.value_ / -value_type(raw_value_type(-b) << 22)};
    } else {
      return Derived{a.value_ / value_type(raw_value_type(b << 22))};
    }
  }

  template <std::integral T>
  friend constexpr Derived operator/(const T &b, const Derived &a) {
    if (b < 0) {
      return Derived{-value_type(raw_value_type(-b) << 22) / a.value_};
    } else {
      return Derived{value_type(b << 22) / (a.value_)};
    }
  }

private:
  static constexpr uint8_t label_[]{Label...};

  value_type value_{};

  static_assert(sizeof(value_type) == 4);
};

class Power : public UnitBase<Power, 'W'> {
public:
  using Base = UnitBase<Power, 'W'>;
  using Base::Base;
  using Base::operator=;
};

class Celcius : public UnitBase<Celcius, u'°', 'C'> {
public:
  using Base = UnitBase<Celcius, u'°', 'C'>;
  using Base::Base;
  using Base::operator=;
};

class Milliseconds : public UnitBase<Milliseconds, 'm', 's'> {
public:
  using Base = UnitBase<Milliseconds, 'm', 's'>;
  using Base::Base;
  using Base::operator=;
};

consteval Voltage operator""_V(long double v) {
  return au::volts(static_cast<uint32_t>(v));
}

constexpr Voltage operator""_V(unsigned long long v) { return au::volts(v); }

consteval Voltage operator""_mV(long double v) {
  return au::milli(au::volts)(static_cast<uint32_t>(v));
}

constexpr Voltage operator""_mV(unsigned long long v) {
  return au::milli(au::volts)(static_cast<uint32_t>(v));
}
consteval Voltage operator""_uV(long double v) {
  return au::micro(au::volts)(static_cast<uint32_t>(v));
}

constexpr Voltage operator""_uV(unsigned long long v) {
  return au::micro(au::volts)(static_cast<uint32_t>(v));
}

constexpr float_type operator""_sf(unsigned long long v) {
  return static_cast<uint32_t>((v << 20u));
}

consteval float_type operator""_sf(long double v) {
  return static_cast<double>(v);
}

constexpr bool f() {
  Voltage v1 = au::volts(10);
  auto v = v1.in(au::micro(au::volts));
  Current i1 = au::amperes(2);
  Resistance r1 = (v1 / au::unblock_int_div(i1)).as(au::ohms);
  return (r1.in(au::milli(au::ohms)) == 5000);
}
static_assert(f());
#endif
