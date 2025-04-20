#ifndef CLI_FORMAT_HPP
#define CLI_FORMAT_HPP

#include "cli/enums.hpp"
#include "cli/util.hpp"

namespace cli::format {
struct FormatResult {
  constexpr FormatResult(Error error) : error(error), size_written(0) {}
  constexpr FormatResult(std::size_t size_written)
      : error(Error::none), size_written(size_written) {}

  Error error;
  std::size_t size_written;
};

template <class F> struct formatter_value_type {
  using type = std::decay_t<type_list::type_at_t<
      1, typename function_traits<std::decay_t<F>>::arguments>>;
};

template <typename F, typename T>
concept FormatterOf = requires(F &&f, std::span<uint8_t> buf, const T &t) {
  { f(buf, t) } -> std::same_as<FormatResult>;
};

template <class F>
concept Formatter =
    not std::same_as<void, typename formatter_value_type<F>::type>;

template <class T> struct DefaultFormat {
  constexpr FormatResult operator()(std::span<uint8_t> buf, const T &t) const {
    return Error::unimplemented;
  }
};

struct NullFormat {
  constexpr FormatResult operator()(std::span<uint8_t> buf,
                                    const dummy &) const {
    return 0;
  }
};

template <> struct DefaultFormat<void> {
  constexpr FormatResult operator()(std::span<uint8_t> buf) const {
    return Error::unimplemented;
  }
};

} // namespace cli::format

#endif
