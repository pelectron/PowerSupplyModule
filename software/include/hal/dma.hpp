#ifndef HAL_DMA_HPP
#define HAL_DMA_HPP

#include "hal/callback.hpp"
#include "hal/enums.hpp"
#include "tl/expected.hpp"
#include <variant>
namespace hal::dma {

struct Block {
  void *data;
  std::size_t size;
};

template <typename T>
tl::expected<Block, Error> make_block(T *data, std::size_t size) {}

struct Transfer {
  template <typename T>
  Transfer(T *data_src, std::size_t src_size, T *data_dest,
           Mode m = Mode::single, Priority prio = Priority::low);
  template <typename T>
  Transfer(T *data_src, std::size_t src_size, Peripheral dest,
           Mode m = Mode::single, Priority prio = Priority::low);
  template <typename T>
  Transfer(Peripheral source, T *dest, std::size_t dest_size,
           Mode m = Mode::single, Priority prio = Priority::low);

  std::variant<Peripheral, Block> src;
  std::variant<Peripheral, Block> dst;
  DataSize size;
  Mode mode;
  Priority priority;
  std::uint8_t channel;
};

using Callback = hal::Callback<16, 4, hal::Error>;

Error initiate(const Transfer &transfer, Callback &callback);
} // namespace hal::dma

#endif
