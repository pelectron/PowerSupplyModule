#ifndef MOCK_HAL_I2C_HPP
#define MOCK_HAL_I2C_HPP
#include "cli/ctti.hpp"
#include "hal/enums.hpp"
#include "hal/i2c.hpp"

#include <iostream>
#include <vector>
namespace hal::i2c {

struct MockHandle {

  hal::Error write(uint8_t address, std::span<const uint8_t> buffer) {
    std::cout << "i2c-write [" << cli::ctti::enum_name(config.id)
              << "] addr: " << unsigned{address} << " size: " << buffer.size()
              << " data: [";
    for (std::size_t i = 0; i < buffer.size(); ++i) {
      write_input[i] = buffer[i];
      std::cout << unsigned{read_response[i]};
      if (i != buffer.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    return hal::Error::none;
  }

  hal::Error read(uint8_t address, std::span<uint8_t> buffer) {
    std::cout << "i2c-read [" << cli::ctti::enum_name(config.id)
              << "] addr: " << unsigned{address} << " size: " << buffer.size()
              << " data: [";

    if (buffer.size() > read_response.size())
      read_response.resize(buffer.size(), 0);

    for (std::size_t i = 0; i < buffer.size(); ++i) {
      buffer[i] = read_response[i];
      std::cout << unsigned{read_response[i]};
      if (i != buffer.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    return hal::Error::none;
  }

  hal::i2c::Config config;
  std::vector<std::uint8_t> read_response;
  std::vector<std::uint8_t> write_input;
};

} // namespace hal::i2c
#endif
