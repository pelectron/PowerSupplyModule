#ifndef PSM_PARSE_HPP
#define PSM_PARSE_HPP

#include "result.hpp"
#include <span>

namespace psm::fmt {

template <typename T> struct ParseResult {
  Result<T> value;
  const char *ptr;
};

template <typename T> ParseResult<T> parse(std::span<const char> str);

template <typename T>
Result<uint32_t> format_to(std::span<char> buf, const T &value);

} // namespace psm::fmt

#endif
