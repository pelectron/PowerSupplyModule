#ifndef HAL_UART_HPP
#define HAL_UART_HPP

#include "hal/config.hpp"
#include "hal/enums.hpp"
#include "hal/operations.hpp"
#include "poly.hpp"
#include "poly/fwd.hpp"

#include <concepts>
#include <cstdint>
#include <span>
#include <type_traits>

namespace hal::uart {

class Device;

using ListenerFunction = void (*)(Device *obj, char c);

struct NullHandle {
  constexpr hal::Error write(std::span<const uint8_t>) {
    return hal::Error::none;
  }

  constexpr hal::Error read(std::span<uint8_t>) { return hal::Error::none; }

  hal::Error register_callback(Device *, ListenerFunction) {
    return hal::Error::none;
  }
};

template <class Storage>
using Handle = poly::Struct<
    Storage, poly::type_list<>,
    poly::type_list<hal::Error(hal::write, std::span<const uint8_t> buffer),
                    hal::Error(hal::read, std::span<uint8_t> buffer),
                    hal::Error(hal::register_callback, Device *device,
                               ListenerFunction callback)>>;

using HandleOwner = Handle<poly::move_only_local_storage<72>>;

using HandleRef = Handle<poly::ref_storage>;

class Device {
public:
  constexpr Device() = default;
  constexpr Device(Device &&) = default;
  constexpr explicit Device(HandleRef handle, int) : handle_(handle) {}
  constexpr Device &operator=(Device &&) = default;

  enum Error {
    none,
    invalid_handle,
    buffer_overflow,
    buffer_underflow,
    io_error
  };

  hal::Error put(char c) { return handle_.write({(const uint8_t *)&c, 1}); }

  hal::Error put(const char *s, std::size_t size) {
    return handle_.write({(const uint8_t *)s, size});
  }

  template <std::invocable<const char *, std::size_t> OnMessage>
  void set_on_message(OnMessage &&on_msg) {
    // on_msg_ = std::forward<OnMessage>(on_msg);
  }

  constexpr hal::Error write(std::span<const uint8_t> buffer) {
    if (not handle_)
      return hal::Error::invalid_handle;
    return handle_.write(buffer);
  }

  constexpr hal::Error read(std::span<uint8_t> buffer) {
    if (not handle_)
      return hal::Error::invalid_handle;
    return handle_.read(buffer);
  }

  template <std::invocable<char> Callback>
  constexpr hal::Error register_callback(Callback &&cb) {
    using Cb = std::remove_cvref_t<Callback>;

    callback.emplace<Cb>(std::forward<Callback>(cb));
    return handle_.register_callback(
        this, +[](Device *self, char c) {
          auto cb = reinterpret_cast<Cb *>(self->callback.data());
          if (cb != nullptr)
            (*cb)(c);
        });
  }

private:
  HandleRef handle_;
  poly::move_only_local_storage<32> callback;
};

ConfigResult<Device> configure(const Config &cfg) noexcept;
} // namespace hal::uart

#endif
