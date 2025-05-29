
// #include "cli/cli.hpp"
#include "drivers/tps55288.hpp"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "hal/stm32c031xx/clocks.hpp"
#include "hal/uart.hpp"

extern "C" {
void SystemInit(void) {}
int _exit(int) {}
int _close(int) {}
int _lseek() {}
int _read() {}
int _write(int) {}
int _sbrk_r() {}
}

// static constinit psm::Psm module{};

// static auto psm_cli{psm::cli::make_cli(module, module.uart)};
// static constinit cli::io::Input in(psm_cli);

using namespace hal::gpio;
using namespace hal;
static inline constexpr struct Hw {
  hal::gpio::Config wp{Port::B | Pin9, Function::output};
  hal::gpio::Config nplug{Port::C | Pin14, Function::input};
  hal::gpio::Config wdi{Port::C | Pin15, Function::output};
  hal::gpio::Config nwlat{Port::B | Pin6, Function::output};
  hal::gpio::Config n_en_ldo{Port::C | Pin6, Function::output};
  hal::gpio::Config en_bb{Port::A | Pin9, Function::output, Mode::push_pull,
                          Speed::slow,    Pull::none,       State::reset};
  hal::gpio::Config out_series{Port::A | Pin8,  Function::output,
                               Mode::push_pull, Speed::slow,
                               Pull::none,      State::reset};
  hal::gpio::Config out_en_n{Port::B | Pin2, Function::output, Mode::push_pull,
                             Speed::slow,    Pull::none,       State::reset};
  hal::gpio::Config out_en_p{Port::B | Pin0, Function::output, Mode::push_pull,
                             Speed::slow,    Pull::none,       State::reset};
  hal::gpio::Config out_select{Port::B | Pin1,  Function::output,
                               Mode::push_pull, Speed::slow,
                               Pull::none,      State::reset};
  hal::i2c::Config i2c{.id = hal::i2c::Id::A,
                       .scl = Port::B | Pin8,
                       .sda = Port::B | Pin7,
                       .speed = i2c::Speed::normal,
                       .frequency = 100'000};
  hal::spi::Config spi{.id = spi::Id::A,
                       .sclk = Port::B | Pin3,
                       .mosi = Port::B | Pin5,
                       .miso = Port::B | Pin4,
                       .cs = Port::A | Pin15,
                       .phase = spi::Phase::low,
                       .polarity = spi::Polarity::low,
                       .format = spi::Format::msb_first,
                       .baudrate = 1'000'000u,
                       .data_size = 8,
                       .use_hw_cs = false,
                       .crc = spi::Crc::none,
                       .crc_polynomial = 0,
                       .three_wire = false,
                       .cs_pulse = false};
  hal::uart::Config uart{.id = uart::Id::A,
                         .tx = Port::A | Pin0,
                         .rx = Port::A | Pin1,
                         .baudrate = 115200,
                         .bits = uart::Bits::eight,
                         .stop_bits = uart::StopBits::one,
                         .parity = uart::Parity::none};
} hw;

static constinit spi::Device spi_dev;
static constinit i2c::Device i2c_dev;

static gpio::Pin conf(const gpio::Config &cfg) {
  auto res = hal::gpio::configure(cfg);
  if (not res) {
    static constexpr std::uint8_t buf[]{"failed to intialize pin!"};
    while (1) {
      volatile int i = 0;
      while (i < 1000000) {
        i = i + 1;
      }
    }
  }
  return res.peripheral;
}

volatile int i = 0;
volatile hal::ConfigError config_err;
volatile hal::Error error;
int main() {
  *reinterpret_cast<volatile std::uint32_t *>(0xE000ED0Cu) |= 0x08000000UL;
  stm32c031xx::clock_tree.init();
  uart::Device uart_dev;
  // TODO: fix copy and move operations for devices
  if (auto res = hal::uart::configure(hw.uart); not res) {
    config_err = res.error;
    while (i != 1) {
    }
  } else {
    uart_dev = std::move(res.peripheral);
  }

  if (auto res = hal::spi::configure(hw.spi); not res) {
    config_err = res.error;
    while (i != 1) {
    }
  } else {
    spi_dev = std::move(res.peripheral);
  }

  if (auto res = hal::i2c::configure(hw.i2c); not res) {
    config_err = res.error;
    while (1) {
    }
  } else {
    i2c_dev = i2c::Device(res.peripheral, 0x64);
  }

  // Pin wp = conf(hw.wp);
  // Pin nplug = conf(hw.nplug);
  // Pin wdi = conf(hw.wdi);
  // Pin nwlat = conf(hw.nwlat);
  // Pin n_en_ldo = conf(hw.n_en_ldo);
  Pin en_bb = conf(hw.en_bb);
  Pin out_series = conf(hw.out_series);
  Pin out_en_n = conf(hw.out_en_n);
  Pin out_en_p = conf(hw.out_en_p);
  Pin out_select = conf(hw.out_select);
  tps55288::TPS55288 buck{i2c_dev, std::move(en_bb)};
  error = buck.enable();
  error = buck.enable_ilim(true);
  auto r = buck.register_map();
  // if (s == hal::gpio::State::set) {
  const uint8_t b[]{"hello\n"};
  // } else {
  //   const uint8_t b[]{"world"};
  // }
  // psm::Error e = psm::Error::none;
  // module.init(psm::get_psm(), hal::get_hal(), cli::io::Input{});

  // while (e == psm::Error::none) {
  //   e = module.loop();
  // }
  //
  // module.reset(e);

  out_series.set(State::reset);
  out_en_n.set(State::reset);
  out_en_p.set(State::reset);
  out_select.set(State::reset);
  uart_dev.write({b, 6});
  while (1) {
    while (i < 10000000)
      i += 1;
    i = 0;
    // wp.toggle();
    // wdi.toggle();
    // nplug.set(State::set);
    // nwlat.set(State::set);
    // n_en_ldo.set(State::set);
    // en_bb.set(State::reset);
    // out_series.toggle();
    // out_en_n.toggle();
    // out_en_p.toggle();
    // out_select.toggle();
    uart_dev.write({b, 6});
  }

  return 0;
}
