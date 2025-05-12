
#include "cli/cli.hpp"
#include "com.hpp"
#include "error.hpp"
#include "hal/hal.hpp"
#include "psm.hpp"

// extern "C" {
// void SystemInit(void) {}
// int _exit(int) {}
// int _close(int) {}
// int _lseek() {}
// int _read() {}
// int _write(int) {}
// int _sbrk_r() {}
// }
// namespace std {
// void __glibcxx_assert_fail(char const *, int, char const *,
//                            char const *) noexcept {
//   while (1) {
//   }
// }
// } // namespace std

static constinit psm::Psm module{};

static auto psm_cli{psm::cli::make_cli(module, module.uart)};
static constinit cli::io::Input in(psm_cli);

int main() {

  psm::Error e =
      module.init(psm::get_psm(), hal::get_hal(), cli::io::Input(psm_cli));

  if (e != psm::Error::none) {
    while (1) {
    }
  }

  while (e == psm::Error::none) {
    e = module.loop();
  }

  module.reset(e);

  while (1) {
  }

  return 0;
}
