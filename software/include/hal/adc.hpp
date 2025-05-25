#ifndef HAL_ADC_HPP
#define HAL_ADC_HPP

#include "hal/config.hpp"
#include "hal/operations.hpp"

#include "poly.hpp"
#include "tl/expected.hpp"
#include "units.hpp"
#include <array>
#include <cstdint>

namespace hal::adc {
constexpr Voltage code_to_voltage(std::int32_t code, Voltage vref,
                                  unsigned num_bits) {
  return code * (vref / (1 << num_bits));
}

class Channel;
class Adc;

template <poly::Storage S>
using Handle = poly::Struct<
    S, poly::type_list<>,
    poly::type_list<Error(enable), Error(disable), Error(start, ChannelId),
                    Error(stop, ChannelId),
                    hal::Error(sample, ChannelId channels, Voltage *buf,
                               std::size_t bufsize),
                    tl::expected<Channel, hal::Error>(init, Adc &adc,
                                                      const ChannelConfig &cfg),
                    Error(deinit, ChannelId)>>;

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
        config_(std::exchange(other.config_, Config{})), samples_{} {}

  constexpr Adc(HandleRef handle, Config config)
      : handle_(handle), config_(config) {
    for (const auto &ch : config_.channel_configs) {
      if (ch.channel != ChannelId::invalid) {
        ++num_samples;
      }
    }
  }
  constexpr hal::Error enable() { return handle_.enable(); }

  constexpr hal::Error disable() { return handle_.disable(); }

  /// performs a conversion for all channels. If the adc is configured in
  /// continous mode, then this will start the conversions. If configured for
  /// single convserion, this will perform a single conversion.
  constexpr hal::Error sample() {
    Error e = handle_.start(AllChannels);
    if (e != Error::none)
      return e;

    e = handle_.sample(AllChannels, samples_.data(), num_samples);

    if ((config_.options & Options::continous) == Options::continous) {
      return e;
    } else {
      Error e2 = handle_.stop(AllChannels);
      return e != Error::none ? e : e2;
    }
  }

  constexpr hal::Error stop() { return handle_.stop(AllChannels); }

  tl::expected<Channel, hal::Error> get_channel(ChannelId id) {
    if (not handle_)
      return tl::unexpected(Error::invalid_handle);
    std::size_t index = 0;
    for (const auto &ch : config_.channel_configs) {
      if (index == num_samples)
        break;
      if (id == ch.channel) {
        return Channel(*this, ch, samples_.data() + index);
      }
      ++index;
    }
    return tl::unexpected(Error::invalid_param);
  }

  tl::expected<Voltage, hal::Error> get_voltage(ChannelId id) {
    std::size_t index = 0;
    for (const auto &ch : config_.channel_configs) {
      if (index == num_samples)
        break;
      if (id == ch.channel) {
        return samples_[index];
      }
      ++index;
    }
    return tl::unexpected(Error::invalid_param);
  }

  constexpr ~Adc() {
    if (handle_)
      handle_.deinit(AllChannels);
  }

private:
  friend class Channel;
  HandleRef handle_{};
  Config config_{};
  std::array<Voltage, 16> samples_{};
  std::uint8_t num_samples = 0;
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

  Error e = adc_->handle_.start(config_->channel);
  if (e != Error::none)
    return tl::unexpected(e);

  e = adc_->handle_.sample(config_->channel, buffer_, 1);
  Error e2 = adc_->handle_.stop(config_->channel);

  if (e != Error::none)
    return tl::unexpected(e);
  if (e2 != Error::none)
    return tl::unexpected(e2);

  return *buffer_;
}

inline Channel::~Channel() {
  if (adc_ == nullptr or not adc_->handle_)
    return;

  adc_->handle_.deinit(config_->channel);
}

} // namespace hal::adc
#endif
