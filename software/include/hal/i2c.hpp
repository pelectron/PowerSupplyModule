#ifndef HAL_I2C_HPP
#define HAL_I2C_HPP

#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/operations.hpp"
#include "poly.hpp"

#include <cstdint>
#include <span>

namespace hal::i2c {

struct NullHandle {
  constexpr hal::Error write(uint8_t, std::span<const uint8_t>) {
    return hal::Error::none;
  }

  constexpr hal::Error read(uint8_t, std::span<uint8_t>) {
    return hal::Error::none;
  }
};

template <class Storage>
using Handle =
    poly::Struct<Storage, poly::type_list<>,
                 poly::type_list<hal::Error(hal::write, uint8_t address,
                                            std::span<const uint8_t> buffer),
                                 hal::Error(hal::read, uint8_t address,
                                            std::span<uint8_t> buffer)>>;
using HandleOwner = Handle<poly::move_only_local_storage<72>>;
using HandleRef = Handle<poly::ref_storage>;

class Device {
public:
  constexpr Device() = default;

  constexpr Device(HandleRef handle, uint8_t address)
      : handle_(handle), address_(address) {}

  constexpr hal::Error write(std::span<const uint8_t> buffer) {
    return handle_.write(address_, buffer);
  }

  constexpr hal::Error read(std::span<uint8_t> buffer) {
    return handle_.read(address_, buffer);
  }

  constexpr void set_address(uint8_t addr) { address_ = addr; }

  constexpr bool is_valid() const { return static_cast<bool>(handle_); }

private:
  HandleRef handle_{};
  uint8_t address_{};
};

ConfigResult<HandleRef> configure(const Config &cfg) noexcept;
} // namespace hal::i2c

#endif
