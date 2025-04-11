
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

static constinit psm::PSM module{};

static constinit psm::Hardware hardware{};
static constinit psm::NonVolatileStorage nvs;

int main() {
  module.init(nvs);

  while (1) {
    module.loop();
  }
  return 0;
}
