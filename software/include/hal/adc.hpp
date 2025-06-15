#ifndef HAL_ADC_HPP
#define HAL_ADC_HPP

#include "hal/callback.hpp"
#include "hal/config.hpp"
#include "hal/operations.hpp"

#include "poly.hpp"
#include "tl/expected.hpp"
#include "units.hpp"
#include <array>
#include <cstdint>

namespace hal::adc {

enum Event {
  started,
  stopped,
  ready,
  overrun,
  end_of_calibration,
  end_of_sequence,
  end_of_conversion,
  watchdog1,
  watchdog2,
  watchdog3,
  watchdog4,
  watchdog5,
  watchdog6,
  watchdog7,
  watchdog8,
};

using ConversionCallback =
    hal::Callback<16, 4, tl::expected<std::int32_t, Error>>;
using SeqConversionCallback =
    hal::Callback<16, 4, tl::expected<std::span<std::int32_t>, Error>>;
using MonitorCallback = hal::Callback<16, 4, Event>;

struct CallbackTable {
  ConversionCallback end_of_conversion;
  SeqConversionCallback end_of_sequence;
  MonitorCallback monitor;
};

constexpr Voltage code_to_voltage(std::int32_t code, Voltage vref,
                                  unsigned num_bits) {
  return (code * vref) / (1 << num_bits);
}

class Channel;
class Adc;

template <poly::Storage S>
using Handle =
    poly::Struct<S, poly::type_list<>,
                 poly::type_list<Error(enable), Error(disable),
                                 Error(start, const Config &, CallbackTable &,
                                       std::span<std::int32_t>),
                                 Error(trigger), Error(stop)>>;

using HandleRef = Handle<poly::ref_storage>;

class Adc;

class Channel {
  friend class Adc;
  Adc *adc_ = nullptr;
  Voltage *buffer_;
  const ChannelConfig *config_{};
  constexpr Channel(Adc &adc, const ChannelConfig &cfg, Voltage *buffer);

public:
  constexpr Channel() = default;
  constexpr Channel(Channel &&other) noexcept;
  constexpr Channel &operator=(Channel &&other) noexcept;

  inline tl::expected<Voltage, hal::Error> sample();
  inline ~Channel();
};

class Adc {
public:
  constexpr Adc() = default;
  constexpr Adc(Adc &&other)
      : handle_(std::exchange(other.handle_, HandleRef{})),
        config_(std::exchange(other.config_, Config{})), samples_{},
        num_samples_(other.num_samples_) {}

  constexpr Adc(HandleRef handle, Config config)
      : handle_(handle), config_(config) {
    for (std::uint32_t idx = 0; idx < 32; ++idx) {
      if (config.channels & (1u << idx)) {
        ++num_samples_;
      }
    }
  }
  constexpr hal::Error enable() { return handle_.enable(); }

  constexpr hal::Error disable() { return handle_.disable(); }

  /// performs a conversion. If the adc is configured in
  /// continous mode, then this will start the conversions and will not stop
  /// until stop is called. If configured for single convserion, this will
  /// perform a single conversion.
  constexpr hal::Error start() {
    return handle_.start(config_, callbacks, {samples_.data(), num_samples_});
  }

  constexpr hal::Error trigger() { return handle_.trigger(); }

  constexpr hal::Error stop() { return handle_.stop(); }

  tl::expected<Channel, hal::Error> get_channel(ChannelId id) {
    if (not handle_)
      return tl::unexpected(Error::invalid_handle);
    // std::size_t index = 0;
    // for (const auto &ch : config_.channel_configs) {
    //   if (index == num_samples)
    //     break;
    //   if (id == ch.channel) {
    //     return Channel(*this, ch, samples_.data() + index);
    //   }
    //   ++index;
    // }
    return tl::unexpected(Error::invalid_param);
  }

  tl::expected<Voltage, hal::Error> get_voltage(ChannelId id) {
    if ((id & config_.channels) != id)
      return tl::unexpected(hal::Error::invalid_param);

    for (std::size_t idx = 0; idx < 32; ++idx) {
      if (id & (1u << idx)) {
        return code_to_voltage(samples_[idx], config_.vref, config_.num_bits);
      }
    }

    return tl::unexpected(Error::invalid_param);
  }

  std::span<const int32_t> codes() const {
    return {samples_.data(), num_samples_};
  }

  CallbackTable callbacks;

private:
  friend class Channel;
  HandleRef handle_{};
  Config config_{};
  std::array<std::int32_t, 32> samples_{};
  std::uint8_t num_samples_ = 0;
};

constexpr Channel::Channel(Channel &&other) noexcept
    : adc_(std::exchange(other.adc_, nullptr)),
      config_(std::exchange(other.config_, nullptr)) {}

constexpr Channel::Channel(Adc &adc, const ChannelConfig &cfg, Voltage *buffer)
    : adc_(&adc), buffer_(buffer), config_(&cfg) {}

constexpr Channel &Channel::operator=(Channel &&other) noexcept {
  if (adc_ == nullptr) {
    adc_ = std::exchange(other.adc_, nullptr);
    config_ = std::exchange(other.config_, nullptr);
  }
  return *this;
}

inline tl::expected<Voltage, hal::Error> Channel::sample() {
  if (adc_ == nullptr or not adc_->handle_)
    return tl::unexpected(Error::invalid_handle);

  if ((adc_->config_.options & Options::continous) == Options::continous) {
    return *buffer_;
  }

  // Error e = adc_->handle_.start(config_->channel);
  // if (e != Error::none)
  //   return tl::unexpected(e);
  //
  // e = adc_->handle_.sample(config_->channel, buffer_, 1);
  // Error e2 = adc_->handle_.stop(config_->channel);

  // if (e != Error::none)
  //   return tl::unexpected(e);
  // if (e2 != Error::none)
  //   return tl::unexpected(e2);

  return *buffer_;
}

inline Channel::~Channel() {
  if (adc_ == nullptr or not adc_->handle_)
    return;

  // adc_->handle_.deinit(config_->channel);
}

tl::expected<Adc, hal::ConfigError> configure(const Config &cfg);
} // namespace hal::adc
#endif
