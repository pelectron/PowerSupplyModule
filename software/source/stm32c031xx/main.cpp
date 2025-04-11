#include "com.hpp"
#include "cortex/core.hpp"
#include "hal/i2c.hpp"
#include "hal/peripheral.hpp"
#include "hal/spi.hpp"
#include "hal/uart.hpp"
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

static constinit psm::PSM api;
static constinit psm::ComSlave com;
static constinit hal::uart::Uart uart;
static constinit hal::spi::Master spi;
static constinit hal::i2c::Master i2c;

using core = cm::M0PLUS<cm::IRQ31>;

static constexpr hal::spi::MasterConfig spi_cfg{};
static constexpr hal::uart::Config uart_cfg{};
static constexpr hal::i2c::Config i2c_cfg{};

int main() {

  core::init(48'000'000 / 1'000);
  // setup clocks

  // setup periperals
  // SPI
  if (auto res = hal::spi::configure(spi_cfg))
    spi = std::move(res.peripheral);

  // I2C
  if (auto res = hal::i2c::configure(i2c_cfg))
    i2c = std::move(res.peripheral);

  // UART
  if (auto res = hal::uart::configure(uart_cfg))
    uart = std::move(res.peripheral);

  // setup the psm
  api.init();
  com.init(api);

  while (1) {
  }

  return 0;
}
