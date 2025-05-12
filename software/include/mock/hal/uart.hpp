#ifndef MOCK_HAL_UART_HPP
#define MOCK_HAL_UART_HPP

#include "cli/ctti.hpp"
#include "hal/uart.hpp"

#include <iostream>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "cpp-terminal/exception.hpp"
#include "cpp-terminal/input.hpp"
#include "cpp-terminal/iostream.hpp"
#include "cpp-terminal/key.hpp"
#include "cpp-terminal/options.hpp"
#include "cpp-terminal/terminal.hpp"
#include "cpp-terminal/tty.hpp"
#include "cpp-terminal/version.hpp"

namespace hal::uart {

struct MockHandle {

  hal::Error write(std::span<const uint8_t> buffer) {
    std::cout << "uart-write [" << cli::ctti::enum_name(config.id)
              << "] size: " << buffer.size() << " data: [";
    for (std::size_t i = 0; i < buffer.size(); ++i) {
      write_input[i] = buffer[i];
      std::cout << unsigned{read_response[i]};
      if (i != buffer.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    return hal::Error::none;
  }

  hal::Error read(std::span<uint8_t> buffer) {
    std::cout << "uart-read [" << cli::ctti::enum_name(config.id)
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

  hal::Error register_callback(Device *device, ListenerFunction callback) {
    callbacks[device].push_back(callback);
    return Error::none;
  }

  static inline std::map<Device *, std::vector<ListenerFunction>> callbacks{};

  Config config;
  std::vector<std::uint8_t> read_response;
  std::vector<std::uint8_t> write_input;
};

struct TerminalHandle {

  hal::Error write(std::span<const uint8_t> buffer) {
    for (char c : buffer)
      Term::cout << c;
    Term::cout << std::flush;
    return Error::none;
  }

  hal::Error read(std::span<uint8_t> buffer) {
    for (std::size_t i = 0; i < buffer.size(); ++i) {
      char c = 0;
      Term::cin >> c;
      buffer[i] = c;
    }
    return Error::none;
  }

  hal::Error register_callback(Device *device, ListenerFunction callback) {
    callbacks[device].push_back(callback);
    return Error::none;
  }

  void start_reading() {
    input_reader =
        std::jthread([this](std::stop_token token) {
          try {
            // check if the terminal is capable of handling input
            Term::terminal.setOptions(Term::Option::NoClearScreen,
                                      Term::Option::NoSignalKeys,
                                      Term::Option::Cursor, Term::Option::Raw);
            if (!Term::is_stdin_a_tty()) {
              throw Term::Exception(
                  "The terminal is not attached to a TTY and "
                  "therefore can't catch user input. Exiting...");
            }
            while (1) {
              if (token.stop_requested())
                return;
              Term::Event event = Term::read_event();
              switch (event.type()) {
              case Term::Event::Type::Key: {
                Term::Key key(event);
                if (key == Term::Key::Ctrl_C)
                  return;
                for (const auto &[dev, cbs] : callbacks)
                  for (auto cb : cbs)
                    (*cb)(dev, key.value);
                break;
              }
              case Term::Event::Type::CopyPaste: {
                std::string key_str(event);
                if (!key_str.empty() && key_str[0] == '\033') {
                  Term::cout
                      << "You discovered a key combination not yet managed by "
                         "cpp-terminal (";
                  for (std::size_t i = 0; i != key_str.size(); ++i) {
                    Term::cout << static_cast<std::int32_t>(key_str[i]) << " ";
                  }
                  Term::cout << ").\nPlease report key combination pressed to "
                             << Term::homepage() << std::endl;
                }
              }
              default:
                break;
              }
            }
          } catch (const Term::Exception &re) {
            Term::cerr << "cpp-terminal error: " << re.what() << std::endl;
            return;
          } catch (...) {
            Term::cerr << "Unknown error." << std::endl;
            return;
          }
        });
  }

  void stop_reading() {
    input_reader.request_stop();
    input_reader.join();
  }

  static inline std::jthread input_reader{};
  static inline std::map<Device *, std::vector<ListenerFunction>> callbacks{};
};
} // namespace hal::uart
#endif
