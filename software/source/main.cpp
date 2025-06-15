
// #include "cli/cli.hpp"
#include "cortex/common.hpp"
#include "cortex/core.hpp"
#include "drivers/ad5293.hpp"
#include "drivers/at24cs08.hpp"
#include "hal/adc.hpp"
#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/gpio.hpp"
#include "hal/i2c.hpp"
#include "hal/spi.hpp"
#include "hal/stm32c031xx/clocks.hpp"
#include "hal/uart.hpp"
#include "tl/expected.hpp"

extern "C" {
void SystemInit(void) {}
int _exit(int) {}
int _close(int) {}
int _lseek() {}
int _read() {}
int _write(int) {}
int _sbrk_r() {}
}

using core = cm::M0PLUS<cm::IRQ31>;
using nvic = typename core::nvic;

// static constinit psm::Psm module{};

// static auto psm_cli{psm::cli::make_cli(module, module.uart)};
// static constinit cli::io::Input in(psm_cli);

using namespace hal::gpio;
using namespace hal;
static inline constexpr struct Hw {
  hal::gpio::Config wp{Port::B | Pin9, Function::output, Mode::push_pull,
                       Speed::slow,    Pull::none,       State::set};
  hal::gpio::Config nplug{Port::C | Pin14, Function::input, Mode::none,
                          Speed::slow,     Pull::none,      State::x};
  hal::gpio::Config wdi{Port::C | Pin15, Function::output, Mode::push_pull,
                        Speed::slow,     Pull::none,       State::set};
  hal::gpio::Config nwlat{Port::B | Pin6, Function::output, Mode::push_pull,
                          Speed::slow,    Pull::none,       State::set};
  hal::gpio::Config n_en_ldo{Port::C | Pin6, Function::output, Mode::push_pull,
                             Speed::slow,    Pull::none,       State::set};
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
  hal::adc::Config adc{
      .id = adc::Id::A,
      .channels = adc::Channel2 | adc::Channel3 | adc::Channel4 |
                  adc::Channel5 | adc::Channel6,
      .options = adc::Options::continous | adc::Options::sequence_conversion,
      .num_bits = 12,
      .clock_rate = 48'000'000u,
      .oversampling = 4,
      .sampling_time = 12,
      .vref = au::milli(au::volts)(3000u)};
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
constinit at24cs08::SerialNumber sn;
int main() {
  // 12
  nvic::init();
  nvic::irq_enable(cm::IRQ12);
  *reinterpret_cast<volatile std::uint32_t *>(0xE000ED0Cu) |= 0x08000000UL;
  stm32c031xx::clock_tree().init();
  uart::Device uart_dev;
  at24cs08::AT24CS08 storage{};
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
  auto res = hal::i2c::configure(hw.i2c);
  if (not res) {
    config_err = res.error;
    while (i != 1) {
    }
  }
  auto wp = conf(hw.wp);
  storage.init(res.peripheral, wp, false);
  storage.serial_number()
      .and_then([](at24cs08::SerialNumber s) -> tl::expected<void, hal::Error> {
        sn = s;
        return {};
      })
      .or_else([](hal::Error e) { error = e; });

  auto ad_res = adc::configure(hw.adc);
  if (not ad_res) {
    config_err = ad_res.error();
  }
  auto ad = std::move(ad_res).value();
  ad.callbacks.end_of_conversion.emplace(
      [](tl::expected<std::int32_t, hal::Error> code) {
        while (i < 10)
          i += 1;
        return;
      });
  ad.callbacks.end_of_sequence.emplace(
      [](tl::expected<std::span<std::int32_t>, hal::Error> code) {
        while (i < 20)
          i += 1;
        return;
      });
  // error = ad.enable();
  error = ad.start();
  // error = ad.stop();
  ad5293::AD5293 resistor{std::move(spi_dev)};
  error = resistor.enable();
  error = resistor.wiper(0);
  error = resistor.wiper(1023);
  error = resistor.disable();
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
