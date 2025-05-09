#ifndef CLI_IO_HPP
#define CLI_IO_HPP

#include "cli/enums.hpp"
#include "cli/util.hpp"
#include "cpp-terminal/key.hpp"
#include <cstdint>
#include <type_traits>
#include <utility>
namespace cli::io {

enum class Type : uint8_t {
  AutoComplete,
  BackSpace,
  Char,
  CursorUp,
  CursorDown,
  CursorLeft,
  CursorRight,
  NewLine,
  EraseInDisplay,
  EraseInLine,
  ScrollUp,
  ScrollDown,
};

struct Event {
  Type type;
  uint8_t data[2]{};
  uint8_t final;
};

static_assert(sizeof(Event) == 4);

template <class T, std::size_t Size> class Queue {
  T data[Size + 1];
  T *head;
  T *tail;
};

/**
 * @brief A CharDevice is used to write a single character to an unbuffered
 * output stream.
 * It is a callable that takes a uint8_t character and returns a cli::Error to
 * indicate write success.
 *
 * @tparam D the device type
 */
template <class D>
concept CharStream = requires(std::remove_cvref_t<D> &dev, uint8_t c) {
  { std::invoke(dev, c) } -> std::same_as<Error>;
};

/**
 * @brief A StringDevice is used to write a string of characters to an
 * unbuffered output stream. It is a callable that takes a cli::ByteView
 * character and returns a cli::Error to indicate write success.
 *
 * @tparam D the device type
 */
template <class D>
concept StringStream = requires(std::remove_cvref_t<D> &dev, ByteView s) {
  { std::invoke(dev, s) } -> std::same_as<Error>;
};

/**
 * @brief A BasicOutputDevice is used to write raw characters to an unbuffered
 * output stream.
 *
 * @tparam D the device type
 */
template <class D>
concept BasicOutputStream = CharStream<D> or StringStream<D>;

/**
 * @brief An OutputDevice is the interface cli::io::Output uses to write
 * to an unbuffered output stream.
 *
 * See cli::io::AnsiOutputHandler for an example of an OutputHandler.
 *
 * @tparam H
 */
template <class S>
concept OutputStream = requires(std::remove_cvref_t<S> &s, uint8_t c,
                                ByteView str, const Event &ev) {
  { s.write(c) } -> std::same_as<Error>;
  { s.write(str) } -> std::same_as<Error>;
  { s.event(ev) } -> std::same_as<Error>;
};

/**
 * @brief The output device for ansi terminals
 *
 * @tparam Error
 * @param c
 * @return
 */
template <BasicOutputStream Stream> struct AnsiOutputHandler {
  Stream stream;

  constexpr Error write(uint8_t c);
  constexpr Error write(ByteView s);
  constexpr Error event(const Event &e);
};

template <BasicOutputStream Stream>
AnsiOutputHandler(Stream &&) -> AnsiOutputHandler<std::remove_cvref_t<Stream>>;

/**
 * @brief This class is the interface used by the cli to control the output
 * @tparam Handler
 */
template <OutputStream Stream> class Output {
public:
  template <OutputStream S>
  constexpr Output(S &&stream) : stream(std::forward<S>(stream)) {}
  template <BasicOutputStream S>
  constexpr Output(S &&stream) : stream(std::forward<S>(stream)) {}

  /// write a character to the terminal
  constexpr Error write(uint8_t c);

  /// write a string to the terminal
  constexpr Error write(ByteView s);

  /// delete the last n characters
  constexpr Error backspace(uint8_t n = 1);
  /// move cursor up by n
  constexpr Error cursor_up(uint8_t n = 1);
  /// move cursor down by n
  constexpr Error cursor_down(uint8_t n = 1);
  /// move cursor left by n
  constexpr Error cursor_left(uint8_t n = 1);
  /// move cursor right by n
  constexpr Error cursor_right(uint8_t n = 1);

  constexpr Error erase_in_display(uint8_t n = 1);
  constexpr Error erase_in_line(uint8_t n = 1);
  constexpr Error newline();
  constexpr Error scroll_up(uint8_t n = 1);
  constexpr Error scroll_down(uint8_t n = 1);

private:
  Stream stream;
};

template <OutputStream Stream>
Output(Stream &&) -> Output<std::remove_cvref_t<Stream>>;

template <BasicOutputStream Stream>
Output(Stream &&) -> Output<AnsiOutputHandler<std::remove_cvref_t<Stream>>>;

/**
 * @class Input
 * @brief This class represents an input device, e.g. a keyboard or UART input
 * stream, and is used to send events to the cli.
 *
 * It processes data character by character and calls the clis put_event()
 * function.
 *
 * Function:
 * - handle special ascii characters (DEL/BS/ESC/tabs/feeds)
 * - handle a basc set of ansi escape sequences
 *
 * To use it, construct it with a reference to the cli and call on_char() in
 * your input stream receive callback, for example your UART handler.
 *
 * Example:
 *
 * ```
 * // the cli instance
 * constinit cli::Cli cli(...);
 *
 * // the input device
 * constinit cli:io::Input in(cli);
 *
 * // the callback function used by your HAL
 * void UART_RxCallback(Handle_t* h){
 *   cli::Error error = in.on_char(UART_GetChar(h));
 *   switch(error){
 *      case Error::none: ...
 *      case Error::buffer_overflow: ...
 *      case Error::invalid_esc_seq: ...
 *      default: ...
 *   }
 * }
 * ```
 */
class Input {
public:
  template <class Cli>
  constexpr Input(Cli &cli)
      : cli(&cli), put_event_(+[](void *cli, const Event &ev) {
          return static_cast<Cli *>(cli)->put_event(ev);
        }) {}

  constexpr Error autocomplete();
  constexpr Error backspace(uint8_t n = 1);
  constexpr Error cursor_up(uint8_t n = 1);
  constexpr Error cursor_down(uint8_t n = 1);
  constexpr Error cursor_left(uint8_t n = 1);
  constexpr Error cursor_right(uint8_t n = 1);
  constexpr Error newline();
  constexpr Error on_char(uint8_t c);

private:
  constexpr Error parse_seq_and_put_event();

  enum class State : uint8_t {
    normal,
    esc_sequence,
    esc_sequence_param,
    esc_sequence_intermediate
  };

  uint8_t buffer[12]{};
  uint8_t size = 0;
  uint8_t param_start = 0;
  uint8_t param_end = 0;
  State state = State::normal;
  void *cli;
  Error (*put_event_)(void *, const Event &);
};

namespace dtl {
using Pair = std::pair<uint8_t, Event>;

static constexpr Event esc_code_map[] = {
    {Type::CursorUp, {1, 0}, 'A'},       {Type::CursorDown, {1, 0}, 'B'},
    {Type::CursorRight, {1, 0}, 'C'},    {Type::CursorLeft, {1, 0}, 'D'},
    {Type::EraseInDisplay, {0, 0}, 'J'}, {Type::EraseInLine, {0, 0}, 'K'},
    {Type::ScrollUp, {1, 0}, 'S'},       {Type::ScrollDown, {1, 0}, 'T'},
};
} // namespace dtl

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::write(uint8_t c) {
  return stream.write(c);
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::write(ByteView s) {
  return stream.write(s);
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::backspace(uint8_t n) {
  return stream.event(Event{Type::BackSpace, {n, 0}});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::cursor_up(uint8_t n) {
  return stream.event(Event{Type::CursorUp, {n, 0}, 'A'});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::cursor_down(uint8_t n) {
  return stream.event(Event{Type::CursorDown, {n, 0}, 'B'});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::cursor_left(uint8_t n) {
  return stream.event(Event{Type::CursorLeft, {n, 0}, 'D'});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::cursor_right(uint8_t n) {
  return stream.event(Event{Type::CursorDown, {n, 0}, 'C'});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::erase_in_display(uint8_t n) {
  return stream.event(Event{Type::EraseInDisplay, {n, 0}, 'J'});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::erase_in_line(uint8_t n) {
  return stream.event(Event{Type::EraseInDisplay, {n, 0}, 'K'});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::newline() {
  return stream.event(Event{Type::NewLine});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::scroll_up(uint8_t n) {
  return stream.event(Event{Type::ScrollUp, {n, 0}, 'K'});
}

template <OutputStream OutputStream>
constexpr Error Output<OutputStream>::scroll_down(uint8_t n) {
  return stream.event(Event{Type::ScrollDown, {n, 0}, 'K'});
}

template <BasicOutputStream Device>
constexpr Error AnsiOutputHandler<Device>::write(uint8_t c) {
  if constexpr (StringStream<Device>) {
    const char ch[1] = {static_cast<char>(c)};
    return stream(ByteView(ch, 1));
  } else {
    return stream(c);
  }
}

template <BasicOutputStream Device>
constexpr Error AnsiOutputHandler<Device>::write(ByteView s) {
  if constexpr (StringStream<Device>) {
    return stream(s);
  } else {
    for (const auto &ch : s)
      if (auto err = stream(ch); err != Error::none)
        return err;
  }
  return Error::none;
}

template <BasicOutputStream Device>
constexpr Error AnsiOutputHandler<Device>::event(const Event &e) {
  switch (e.type) {
  case Type::AutoComplete:
    return Error::invalid_argument;
  case Type::BackSpace:
    if (auto err = event({Type::CursorLeft, {1}}); err != Error::none)
      return err;
    return event({Type::EraseInLine});
  case Type::NewLine:
    write('\n');
    return Error::none;
  case Type::Char:
    return Error::invalid_argument;
  default:
    break;
  }

  char buffer[7]{"\x1b["};
  // TODO: escape sequences with more than one argument
  uint8_t final = e.final;
  if (final == 0)
    for (const auto &event : dtl::esc_code_map) {
      if (event.type == e.type) {
        final = event.final;
      }
    }

  if (final == 0)
    return Error::invalid_argument;

  unsigned size = 2;
  auto l = e.data[0] % 10;
  auto m = e.data[0] / 10;
  auto h = m / 10;
  m = m % 10;
  if (h != 0)
    buffer[size++] = h;
  if (not(m == 0 and h == 0))
    buffer[size++] = m;
  buffer[size++] = l;
  buffer[size++] = final;
  if constexpr (StringStream<Device>) {
    return stream(ByteView(buffer, size));
  } else {
    for (std::size_t i = 0; i < size; ++i) {
      if (auto err = stream(buffer[i]); err != Error::none)
        return err;
    }
  }
  return Error::none;
}

constexpr Error Input::autocomplete() {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;
  return (*put_event_)(cli, Event{Type::AutoComplete});
}

constexpr Error Input::backspace(uint8_t n) {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;
  return (*put_event_)(cli, Event{Type::BackSpace, {n, 0}, 0});
}

constexpr Error Input::cursor_up(uint8_t n) {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;
  return (*put_event_)(cli, Event{Type::CursorUp, {n, 0}, 'A'});
}

constexpr Error Input::cursor_down(uint8_t n) {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;
  return (*put_event_)(cli, Event{Type::CursorDown, {n, 0}, 'B'});
}

constexpr Error Input::cursor_left(uint8_t n) {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;
  return (*put_event_)(cli, Event{Type::CursorLeft, {n, 0}, 'D'});
}

constexpr Error Input::cursor_right(uint8_t n) {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;
  return (*put_event_)(cli, Event{Type::CursorRight, {n, 0}, 'C'});
}

constexpr Error Input::newline() {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;
  return (*put_event_)(cli, Event{Type::NewLine});
}

constexpr Error Input::on_char(uint8_t c) {
  if (cli == nullptr or put_event_ == nullptr)
    return Error::implementation_error;

  switch (state) {
  case State::normal:
    switch (c) {
    case ansi::ESC:
      // a potential control sequence has started
      state = State::esc_sequence;
      size = 1;
      buffer[0] = c;
      break;
    case ansi::CSI:
      // a control sequence has started
      state = State::esc_sequence_param;
      size = 1;
      buffer[0] = c;
      param_start = 1;
      break;
    case ansi::BS:
      return backspace(1);
    case ansi::DEL:
      return backspace(1);
    case ansi::HT:
      return autocomplete();
    case ansi::CR:
      [[fallthrough]];
    case ansi::LF:
      [[fallthrough]];
    case ansi::FF:
      [[fallthrough]];
    case ansi::VT:
      return newline();
    default:
      return (*put_event_)(cli, Event{Type::Char, {c}});
    }
  case State::esc_sequence:
    if (c != '[') {
      // not a control sequence -> ignore and go back to normal
      state = State::normal;
      return Error::invalid_esc_seq;
    }
    state = State::esc_sequence_param;
    buffer[size++] = c;
    param_start = size;
    return Error::none;
  case State::esc_sequence_param:
    if (c >= 0x30u and c <= 0x3Fu) {
      // a parameter byte -> expand or create params
      buffer[size++] = c;
    } else if (c >= 0x20u and c <= 0x2Fu) {
      // end of params, start of intermediate
      param_end = size;
      buffer[size++] = c;
      state = State::esc_sequence_intermediate;
    } else if (c >= 0x40u and c <= 0x7Eu) {
      // end of sequence, skipped intermediate
      buffer[size++] = c;
      return parse_seq_and_put_event();
    } else {
      // illegal byte -> choose to abort command and report the invalid
      // sequence
      state = State::normal;
      return Error::invalid_esc_seq;
    }
    return Error::none;
  case State::esc_sequence_intermediate:
    if (c >= 0x20u and c <= 0x2Fu) {
      // still an intermediate byte
      buffer[size++] = c;
    } else if (c >= 0x40u and c <= 0x7Eu) {
      // end of sequence
      buffer[size++] = c;
      return parse_seq_and_put_event();
    } else {
      // illegal byte -> choose to abort command and report the invalid
      // sequence
      state = State::normal;
      return Error::invalid_esc_seq;
    }
    return Error::none;
  default:
    return Error::invalid_state;
  }
}

constexpr Error Input::parse_seq_and_put_event() {
  const uint8_t final_byte = buffer[size - 1];
  Event ev{};
  bool found = false;
  for (const auto &event : dtl::esc_code_map) {
    if (event.final == final_byte) {
      ev = event;
      found = true;
      break;
    }
  }

  if (not found) {
    state = State::normal;
    return Error::invalid_esc_seq;
  }

  auto p_idx = param_start;
  if (p_idx != param_end) {
    unsigned data_idx = 0;
    uint8_t val = 0;
    while (p_idx != param_end) {
      const auto ch = buffer[p_idx];
      if (ch >= '0' and ch <= '9') {
        if (val > 25) {
          state = State::normal;
          return Error::invalid_esc_seq;
        }
        val = val * 10 + ch - '0';
        if (p_idx == param_end - 1) {
          if (data_idx > 1) {
            state = State::normal;
            return Error::invalid_esc_seq;
          }
          ev.data[data_idx++] = val;
        }
      } else if (ch == ';') {
        if (data_idx > 1) {
          state = State::normal;
          return Error::invalid_esc_seq;
        }
        if (p_idx == param_start or buffer[p_idx - 1] == ';') {
          // use default alue
          ++data_idx;
        } else {
          // use parsed alue
          ev.data[data_idx++] = val;
        }
        val = 0;
      } else {
        state = State::normal;
        return Error::invalid_esc_seq;
      }
      ++p_idx;
    }
  }
  auto err = (*put_event_)(cli, ev);
  state = State::normal;
  return err;
}
} // namespace cli::io

#endif
