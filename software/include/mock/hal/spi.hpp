#ifndef MOCK_HAL_SPI_HPP
#define MOCK_HAL_SPI_HPP

#include "cli/ctti.hpp"
#include "hal/spi.hpp"

#include <iostream>
#include <vector>
namespace hal::spi {

struct MockHandle {

  hal::Error write(std::span<const uint8_t> buffer) {
    std::cout << "spi-write [" << cli::ctti::enum_name(config.id)
              << "] size: " << buffer.size() << " data: [";
    for (std::size_t i = 0; i < buffer.size(); ++i) {
      write_input[i] = buffer[i];
      std::cout << unsigned{buffer[i]};
      if (i != buffer.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    return hal::Error::none;
  }

  hal::Error read(std::span<uint8_t> buffer) {
    std::cout << "spi-read [" << cli::ctti::enum_name(config.id)
              << "] size: " << buffer.size() << " data: [";

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

  Config config;
  std::vector<std::uint8_t> read_response;
  std::vector<std::uint8_t> write_input;
};

} // namespace hal::spi

#endif
