#ifndef CLI_HISTORY_HPP
#define CLI_HISTORY_HPP
#include "cli/util.hpp"
#include <cstddef>

namespace cli {
template <std::size_t Depth, std::size_t LineSize> class History {
  using String = StringLiteral<LineSize>;
  String lines[Depth];
  std::size_t front_{0};
  std::size_t current_{0};

public:
  constexpr ByteView get() const noexcept { return lines[current_]; }
  constexpr ByteView backward() noexcept {
    if (current_ == 0) {
      if (front_ != Depth - 1) {
        current_ = Depth - 1;
      }
      return lines[current_];
    }

    if (current_ != front_ + 1)
      --current_;
    return lines[current_];
  }
  constexpr ByteView forward() noexcept {
    if (current_ == front_)
      return lines[current_];
    ++current_;
    if (current_ == Depth)
      current_ = 0;
    return lines[current_];
  }
  constexpr void push_back(ByteView line) noexcept {
    if (front_ == Depth - 1)
      front_ = 0;
    else
      ++front_;
    lines[front_] = String(line);
    current_ = front_;
  }
  constexpr void reset() {
    front_ = 0;
    current_ = 0;
    for (auto &line : lines)
      line.clear();
  }
};
} // namespace cli
#endif
