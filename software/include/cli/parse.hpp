#ifndef CLI_PARSE_HPP
#define CLI_PARSE_HPP

#include "cli/enums.hpp"
#include "cli/traits.hpp"
#include "cli/util.hpp"

#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <type_traits>

namespace cli::parse {

template <class T>
concept Value = std::is_object_v<std::decay_t<T>>;

template <class R>
concept Result = requires(R &&r) {
  { r.error } -> std::convertible_to<Error>;
  { r.value } -> Value;
  { r.rest } -> std::convertible_to<ByteView>;
  { static_cast<bool>(r) };
};

template <class P>
concept Parser = requires(P &&p, ByteView str) {
  { p(str) } -> Result;
};

template <Parser P> struct value_type {
  using type = std::decay_t<decltype(std::declval<decltype(std::declval<P>()(
                                         std::declval<ByteView>()))>()
                                         .value)>;
};
template <Parser P> using value_type_t = typename value_type<P>::type;

template <class P, class T>
concept ParserOf = Parser<P> and std::same_as<T, value_type_t<P>>;

template <class T> struct ParseResult {
  constexpr ParseResult(Error error) : error{error}, value{}, rest{} {}
  constexpr ParseResult(const T &value, ByteView rest = {})
      : error{Error::none}, value{value}, rest{rest} {}

  constexpr ParseResult(T &&value, ByteView rest = {})
      : error{Error::none}, value{std::move(value)}, rest{rest} {}

  constexpr operator bool() const noexcept { return error == Error::none; }

  Error error;
  T value;
  ByteView rest;
};

static_assert(Result<ParseResult<int>>);

/**
 * @brief A parser for integers.
 *
 * @tparam T
 */
template <traits::Integer T, Fmt Format = Fmt::normal | Fmt::hex | Fmt::binary>
class Int {
  using traits = cli::traits::integer_traits<T>;
  using type = typename traits::type;

protected:
  static constexpr std::size_t max_hex_length() {
    switch (sizeof(type)) {
    case 1:
      return 2;
    case 2:
      return 4;
    case 4:
      return 8;
    default:
      return 8;
    }
  }
  static constexpr std::size_t max_bin_length() {
    switch (sizeof(type)) {
    case 1:
      return 8;
    case 2:
      return 16;
    case 4:
      return 32;
    default:
      return 32;
    }
  }
  static constexpr std::size_t max_dec_length() {
    switch (sizeof(type)) {
    case 1:
      return 3;
    case 2:
      return 6;
    case 4:
      return 10;
    default:
      return 10;
    }
  }
  constexpr ParseResult<T> parse_hex(ByteView str, std::size_t offset) {
    constexpr auto to_hex = [](uint8_t c) -> uint8_t {
      if (c >= '0' and c <= '9') {
        return c - '0';
      } else if (c >= 'A' and c <= 'F') {
        return c - 'A';
      } else if (c >= 'a' and c <= 'f') {
        return c - 'a';
      } else {
        return 0xFFu;
      }
    };

    const std::size_t max_size = std::min(
        max_hex_length() + offset, str.size()); // the maximum amount of digits
                                                // we can consume with 32 bits.
    std::uint32_t v = 0;
    for (std::size_t i = offset; i < max_size; ++i) {
      auto c = str[i];
      if (auto bits = to_hex(c); bits <= 0x0Fu) {
        v = (v << 4u) | bits;
      } else {
        // invalid character encountered
        return {std::bit_cast<T>(v), str.substr(i)};
      }
    }

    return {std::bit_cast<T>(v), str.substr(max_size)};
  }

  constexpr ParseResult<T> parse_bin(ByteView str, std::size_t offset) {
    const std::size_t max_size = std::min(
        max_bin_length() + offset, str.size()); // the maximum amount of digits
                                                // we can consume with 32 bits.

    std::uint32_t v = 0;
    for (std::size_t i = offset; i < max_size; ++i) {
      auto c = str[i];
      if (c == '0') {
        v = v << 1u;
      } else if (c == '1') {
        v = (v << 1u) | 1u;
      } else {
        // invalid character encountered
        return {std::bit_cast<T>(v), str.substr(i)};
      }
    }
    return {std::bit_cast<T>(v), str.substr(max_size)};
  }

  constexpr ParseResult<T> parse_dec(ByteView str) {
    if constexpr (not traits::is_signed) {
      std::size_t offset = 0;
      if (str[0] == '+') {
        offset = 1;
        if (str.size() == 1)
          return Error::too_few_characters;
      }
      std::uint32_t v = 0;
      const std::size_t max_size =
          std::min(offset + max_dec_length(), str.size());
      for (std::size_t i = offset; i < max_size; ++i) {
        auto ch = str[i];
        if (ch >= '0' and ch <= '9') {
          v = v * 10 + (ch - '0');
        } else {
          return {std::bit_cast<type>(v), str.substr(i)};
        }
      }
      return {std::bit_cast<type>(v), str.substr(max_size)};
    } else {
      std::size_t offset = 0;
      const bool negative = str[0] == '-';
      if (str[0] == '+' or negative) {
        offset = 1;
        if (str.size() == 1)
          return Error::too_few_characters;
      }

      std::uint32_t v = 0;
      const std::uint32_t max_size =
          std::min(offset + max_dec_length(), str.size());
      for (std::size_t i = offset; i < max_size; ++i) {
        const auto ch = str[i];
        if (ch >= '0' and ch <= '9') {
          v = v * 10 + (ch - '0');
        } else {
          if (negative)
            return {-std::bit_cast<type>(v), str.substr(i)};
          else
            return {std::bit_cast<type>(v), str.substr(i)};
        }
      }
      if (negative)
        return {-std::bit_cast<type>(v), str.substr(max_size)};
      else
        return {std::bit_cast<type>(v), str.substr(max_size)};
    }
  }

public:
  constexpr ParseResult<T> operator()(ByteView str) {
    if (str.size() == 0)
      return Error::too_few_characters;

    if constexpr ((Format & Fmt::hex) == Fmt::hex) {
      // a hexadecimal number, starts with 0x, 0X, x, X, or #
      if (str.starts_with("0x") or str.starts_with("0X"))
        return parse_hex(str, 2);
      if (str.find_first_of("xX#") == 0)
        return parse_hex(str, 1);
    }

    if constexpr ((Format & Fmt::binary) == Fmt::binary) {
      // a binary number, starts with 0b, 0B, b, or B
      if (str.starts_with("0b") or str.starts_with("0B"))
        return parse_bin(str, 2);
      if (str.find_first_of("xX#") == 0)
        return parse_bin(str, 1);
    }

    if constexpr ((Format & Fmt::normal) == Fmt::normal) {
      // a decimal number
      return parse_dec(str);
    }

    return Error::invalid_value;
  }
};

/**
 * @brief A parser for strings. Strings in this context are a continous sequence
 * of printable characters, that is characters in the range 0x21 to 0x7E
 * inclusive.
 *
 * Strings may be enclosed in " quotes. The character " can be escaped with
 * backslash.
 *
 * @tparam T
 * @param str
 * @return
 */
template <traits::String T> class String {
  constexpr ParseResult<T> operator()(ByteView str) {
    if (str.size() == 0)
      return Error::too_few_characters;
    if (str[0] == '"') {
      for (std::size_t i = 1; i < str.size(); ++i) {
        const auto ch = str[i];

        if (ch < 0x21u or ch > 0x7E) // ch is a non printable character
          return Error::unescaped_string;

        if (ch == '"' and str[i - 1] != '\\') {
          const auto value = str.substr(1, i - 1);
          return {T(value.data(), value.size()), str.substr(i)};
        }
      }
      return Error::unescaped_string;
    } else {
      for (std::size_t i = 0; i < str.size(); ++i) {
        const auto ch = str[i];
        if (ch < 0x21u or ch > 0x7E or (ch == '"' and str[i - 1] != '\\')) {
          // ch is a non printable character
          const auto value = str.substr(0, i - 1);
          return {T(value.data(), value.size()), str.substr(i)};
        }
      }
      return {T(str.data(), str.size())};
    }
  }
};

template <class T> class Float {
  static_assert(always_false<T>,
                "The flaoting point parser is unimplemented for now");
};

template <traits::FixPoint T, Fmt Format = Fmt::normal | Fmt::hex | Fmt::binary,
          char FixPointSeparator = '.'>
class FixPoint
    : Int<typename traits::fixpoint_traits<T>::raw_value_type, Format> {
  using Base = Int<typename traits::fixpoint_traits<T>::raw_value_type, Format>;
  using traits = traits::fixpoint_traits<T>;

public:
  constexpr ParseResult<T> operator()(ByteView str) {
    if (str.size() == 0)
      return Error::too_few_characters;

    if constexpr ((Format & Fmt::hex) == Fmt::hex) {
      // a hexadecimal number, starts with 0x, 0X, x, X, or #
      if (str.starts_with("0x") or str.starts_with("0X")) {
        if (auto res = Base::parse_hex(str, 2))
          return {T(res.value), res.rest};
        else
          return res.error;
      }
      if (str.find_first_of("xX#") == 0) {
        if (auto res = Base::parse_hex(str, 1))
          return {T(res.value), res.rest};
        else
          return res.error;
      }
    }
    if constexpr ((Format & Fmt::binary) == Fmt::binary) {
      // a binary number, starts with 0b, 0B, b, or B
      if (str.starts_with("0b") or str.starts_with("0B")) {
        if (auto res = Base::parse_bin(str, 2))
          return {T(res.value), res.rest};
        else
          return res.error;
      }
      if (str.find_first_of("bB") == 0) {
        {
          if (auto res = Base::parse_bin(str, 2))
            return {T(res.value), res.rest};
          else
            return res.error;
        }
      }
    }
    if constexpr ((Format & Fmt::normal) == Fmt::normal) {
      // decimal format
      const auto int_res = Base::parse_dec(str);

      if (not int_res)
        return int_res.error;

      if (int_res.rest.size() == 0 or int_res.rest[0] != FixPointSeparator)
        return {T(int_res.value << traits::num_frac_digits), int_res.rest};

      // the fractional part is in base 10^-N=(2*5)^-N=5^-N*2^-N (N == string
      // size)
      // we want to convert it to base 2^-FracDigits k*b10 = i*b2 ->
      // k*b10/b2 = i -> factor to multiply with: 10^-N/2^-FracDigits
      // (5*2)^-N/2^-FracDigits=2^FracDigits/(5*2)^N = 2^(FracDigits-N)/5^N
      // -> frac*( 2^(N-FracDigits) / 5^N) is the number to store.
      std::uint32_t value = 0;
      for (std::size_t i = 0; i < int_res.rest.size(); ++i) {
        const auto ch = int_res.rest[i];
        if (ch <= '0' or ch >= '9') {
          const auto size = i;
          const auto frac =
              (value << (traits::num_frac_digits - size)) / cxpow(5, size);
          return {T((int_res.value << traits::num_frac_digits) + frac),
                  int_res.rest.substr(i)};
        } else {
          value = value * 10 + (ch - '0');
        }
      }
      const auto frac =
          (value << (traits::num_frac_digits - int_res.rest.size())) /
          cxpow(5, int_res.rest.size());
      return {T((int_res.value << traits::num_frac_digits) + frac)};
    }
    return Error::invalid_value;
  }

  template <typename B, typename U> static constexpr B cxpow(B base, U exp) {
    if (exp == 0)
      return T{1};
    T ret = base;
    --exp;
    while (exp != 0) {
      --exp;
    }
    return ret;
  }
};

template <traits::Struct T>
consteval auto generate_regex() {

};

template <traits::Struct T> class Struct {};

template <class T> class DefaultParse {
public:
  constexpr ParseResult<T> operator()(ByteView str) const {
    return Error::unimplemented;
  }
};

template <traits::Integer T> class DefaultParse<T> : public Int<T> {};

template <traits::Float T> class DefaultParse<T> : public Float<T> {};

template <traits::FixPoint T> class DefaultParse<T> : public FixPoint<T> {};

template <traits::String T> class DefaultParse<T> : public String<T> {};

class NullParse {
public:
  constexpr ParseResult<dummy> operator()(ByteView str) const {
    return {dummy{}, str};
  }
};

namespace {
template <class Tuple, std::size_t I, std::size_t... Is>
Error parse_args_impl(Tuple &t, const ArgVector &v,
                      std::index_sequence<I, Is...>) {
  DefaultParse<std::tuple_element_t<I, Tuple>> parser;
  auto res = parser(v[I]);
  if (res.error != Error::none)
    return res.error;
  std::get<I>(t) = res.value;
  if constexpr (sizeof...(Is) == 0) {
    return Error::none;
  } else {
    return parse_args_impl(t, v, std::index_sequence<Is...>());
  }
}
} // namespace

template <class Tuple>
constexpr Error parse_args(Tuple &t, const ArgVector &v) {
  if constexpr (std::tuple_size_v<Tuple> == 0)
    return Error::none;
  else {
    if (v.size() != std::tuple_size_v<Tuple>) {
      return Error::incorrect_num_params;
    }
    return parse_args_impl(
        t, v, std::make_index_sequence<std::tuple_size_v<Tuple>>());
  }
}

} // namespace cli::parse
#endif
