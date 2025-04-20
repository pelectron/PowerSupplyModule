
#include "cortex/core.hpp"
#include "psm.hpp"

extern "C" {
void SystemInit(void) {}
int _exit(int) {}
int _close(int) {}
int _lseek() {}
int _read() {}
int _write(int) {}
int _sbrk_r() {}
}
namespace std {
void __glibcxx_assert_fail(char const *, int, char const *,
                           char const *) noexcept {
  while (1) {
  }
}
} // namespace std

template <char... c> struct string_constant {};

template <typename Char, Char... cs> constexpr auto operator""_sc() {
  return string_constant<cs...>{};
}
constexpr auto s = "hello"_sc;

static psm::PSM module{};

static constinit psm::Hardware hardware{};
static constinit psm::NonVolatileStorage nvs;

int main() {
  module.init(nvs);

  while (1) {
    module.loop();
  }
  return 0;
}
