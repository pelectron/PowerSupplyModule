/**
 * This file provides a CLI enginge to build a remote interface upon a byte
 * stream.
 *
 * The basics:
 * - the cli is a hierarchical tree of commands
 * - a command has the following properties:
 *   - a name: a string to identify the command
 *   - a description: a string to display for help messages
 *   - the function to execute
 *   - and optionally, any amount of sub commands
 *   - the command sequence: this sequence specifies the command to execute
 *   - the arg sequence: this sequence specifies the command's arguments
 * - the root is owned by the Cli structure.
 * - commands with the root as parents are called root commands.
 *
 * c++ syntax:
 *  ``auto my_cli = cli::cli(config, commands...);``
 *  a command:
 *  ``cli::cmd("[any.path.]name", func)``: a "free function" command. This
 * cannot have subcommands.
 *  ``cli::cmd("[any.path.]name", obj_reference, sub_commands...): an "object"
 * command is added. This must have subcommands
 *  ``cli::cmd("[any.path.]name", const_obj_reference, get): a retrievable
 * parameter is added.
 *
 * syntax:
 * - "cmd [args...]": invoke the command "cmd" with optional args
 * - "cmd.sub [args...]"/"cmd sub [args...]": invoke the command "cmd" with
 * optional args
 * - "get obj.property": returns the obj's property's value
 * - "get.obj.property": returns the obj's property's value
 * - "set obj.property arg": set obj's property value to arg
 * - "set.obj.property arg": set obj's property value to arg
 */
#ifndef CLI_CLI_HPP
#define CLI_CLI_HPP
#include "cli/enums.hpp"
#include "cli/function.hpp"
#include "cli/param.hpp"
#include "cli/util.hpp"

#include <array>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace cli {
using funcs::arg;
using funcs::operator""_a;
using funcs::func;
using funcs::mem_fun;
using params::mem_data;
using params::obj;
using params::param;

template <typename F, typename R, typename... Args>
concept InvocableR = std::is_invocable_r_v<R, F, Args...>;

class HistoryView {
  std::span<const uint8_t> *arr_;
  std::size_t capacity_;
  // the head points to the position that was last written to
  std::span<const uint8_t> *head;
  // the tail points to the position that can be read from
  std::span<const uint8_t> *tail;
  size_t size_;

  constexpr std::span<const uint8_t> *incr(std::span<const uint8_t> *p) {
    ++p;
    if (p == (arr_ + capacity_))
      return arr_;
    else
      return p;
  }

public:
  constexpr HistoryView(std::span<const uint8_t> *arr, std::size_t capacity)
      : arr_(arr), capacity_(capacity), head(arr_), tail(arr_), size_(0) {}

  constexpr bool is_full() noexcept { return size_ == capacity_; }
  constexpr bool is_empty() noexcept { return size_ == 0; }

  constexpr std::size_t size() const { return size_; }
  constexpr std::size_t capacity() const { return capacity_; }

  constexpr void push_back(const std::span<const uint8_t> &t) {
    *head = t;
    if (size_ != capacity_) {
      ++size_;
      head = incr(head);
    } else {
      head = incr(head);
      tail = incr(tail);
    }
  }
};

class Status {
  Error error;
  std::size_t size_written;
};

using CommandSignature = Error(ExecType type, const ArgVector &args,
                               OutputIterator out);

template <class Tuple> constexpr Error parse_args(Tuple &t, const ArgVector &v);

struct Deduced {};

/**
 * @brief split s at occurences of c and store them in splits.
 *
 * At maximum Capacity - 1 split characters can be properly handled. splits will
 * be populated as c as is encountered and not be cleared on error.
 *
 * @tparam Capacity the maximum amount of seperated strings s can consists of
 * @param s the string to split
 * @param c the character to split at
 * @param splits the output vector
 * @return Error::too_many_splits if too many separators have been
 * encountered, or Error::none if the operation was successfull.
 *
 */
constexpr Error split_at(ByteView s, char c, VecView<ByteView> &splits) {
  std::size_t last_sep = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == c) {
      if (i != last_sep and
          not splits.push_back(s.substr(last_sep, i - last_sep))) {
        // add the new substring. This is only valid if i != last_sep. If
        // i==last_sep, then two characters with value c have been enountered
        // after another. If an error occured during push_back,
        return Error::too_many_splits;
      }
      last_sep = i + 1;
    }
  }

  if (not splits.push_back(s.substr(last_sep))) {
    return Error::too_many_splits;
  }

  return Error::none;
}
template <template <class...> class L, class... Commands>
constexpr auto generate_names(L<Commands...>)
    -> std::array<ByteView, sizeof...(Commands)> {
  return {Commands::name...};
}

template <class Name> struct DummyParam : public CommandNode {
  static constexpr ByteView name{Name{}};
  static constexpr ByteView description{"A description"_sc};
  static constexpr ByteView help{"A help"_sc};

  constexpr DummyParam(Name) : CommandNode(*this) {}

  Error execute(ExecType, const ArgVector &, std::span<uint8_t> &) {
    return Error::none;
  }
};

enum class Delimiter { cr, lf, crlf };

enum class Control {
  backspace,
  autocomplete,
  cursor_up,
  cursor_down,
  cursor_left,
  cursor_right,
  insert_char,
  delete_char,
  save_cursor,
  restore_cursor
};

namespace dtl {
/** Escape sequence - Cursor forward (right) */
static const char *escSeqCursorRight = "\x1B[C";

/** Escape sequence - Cursor backward (left) */
static const char *escSeqCursorLeft = "\x1B[D";

/** Escape sequence - Cursor save position */
static const char *escSeqCursorSave = "\x1B[s";

/** Escape sequence - Cursor restore position */
static const char *escSeqCursorRestore = "\x1B[u";

/** Escape sequence - Cursor insert character (ICH) */
static const char *escSeqInsertChar = "\x1B[@";

/** Escape sequence - Cursor delete character (DCH) */
static const char *escSeqDeleteChar = "\x1B[P";

inline constexpr std::pair<Control, ByteView> esc_sequences[]{
    {Control::cursor_right, "\x1B[C"}, {Control::cursor_left, "\x1B[D"},
    {Control::save_cursor, "\x1B[s"},  {Control::restore_cursor, "\x1B[u"},
    {Control::insert_char, "\x1B[@"},  {Control::delete_char, "\x1B[P"},
};

} // namespace dtl

template <typename T>
concept Config = requires(typename std::remove_cvref_t<T>::char_type) {
  {
    std::remove_cvref_t<T>::access_separator
  } -> std::convertible_to<typename std::remove_cvref_t<T>::char_type>;
  {
    std::remove_cvref_t<T>::commands_start_with_separators
  } -> std::convertible_to<bool>;
  { std::remove_cvref_t<T>::tx_size } -> std::convertible_to<std::size_t>;
  { std::remove_cvref_t<T>::rx_size } -> std::convertible_to<std::size_t>;
  {
    std::remove_cvref_t<T>::max_line_length
  } -> std::convertible_to<std::size_t>;
};

struct config {
  // static constexpr std::array<ByteView, sizeof...(Commands)>
  // cmd_names{Commands::name...};
  using char_type = char8_t;
  static constexpr char_type access_separator = '.';
  static constexpr auto command_terminator{"\n"_sc};
  static constexpr bool commands_start_with_separators = true;
  static constexpr std::size_t tx_size = 128;
  static constexpr std::size_t rx_size = 128;
  // TODO: deduced the sizes below
  static constexpr std::size_t max_name_length = 32;
  static constexpr std::size_t max_num_args = 10;
  static constexpr std::size_t max_num_commands = 64;
  static constexpr std::size_t max_cmd_level = 5;
  static constexpr std::size_t max_line_length = 80;
};

struct ControlSequence {
  ByteView introducer;
  ByteView params;
  ByteView intermediate;
  uint8_t final;
  constexpr operator ByteView() const noexcept {
    return {introducer.data(),
            introducer.size() + params.size() + intermediate.size() + 1};
  }
  constexpr void reset() {}
};

template <Config Cfg, Command... Commands> class Cli {
  enum State {
    idle,   // the rx buffer is empty
    active, // rx buffer is not empty, but not while command received
    esc,    // escape was received
    ctrl_seq,
    ctrl_seq_param,
    ctrl_seq_intermediate,
  };

public:
  template <Config Cfg_, Command... Cmds>
  constexpr Cli(Cfg_ &&cfg, Cmds &&...cmds)
      : config_{std::forward<Cfg_>(cfg)},
        commands_{std::forward<Cmds>(cmds)...} {
    init_tree();
  }

  constexpr Cli(Cli &&other)
      : config_{std::move(other.config_)},
        commands_{std::move(other.commands_)} {
    init_tree();
  }

  constexpr Cli(const Cli &other)
      : config_{other.config_}, commands_{other.commands_} {
    init_tree();
  }

  constexpr Cli &operator=(Cli &&other) {
    config_ = std::move(other.config_);
    commands_ = std::move(other.commands_);
    current_cmd = nullptr;
    tx_buf.reset();
    rx_buf.reset();
    current_line_.reset();
    state_ = idle;
    esc_seq_index = 0;
    init_tree();
    return *this;
  }

  constexpr Cli &operator=(const Cli &other) {
    config_ = other.config_;
    commands_ = other.commands_;
    current_cmd = nullptr;
    tx_buf.reset();
    rx_buf.reset();
    current_line_.reset();
    state_ = idle;
    esc_seq_index = 0;
    init_tree();
    return *this;
  }

  constexpr void init_tree() {
    // create the root
    CommandNode &root = cmds_[0];
    root.name = "root";
    root.description = "root";
    // add its sub commands
    std::size_t index = 0;
    cli::for_each([&index, &root,
                   this](auto &cmd) { init_cmds(*this, ++index, root, cmd); },
                  commands_);
  }

  template <Command Cmd>
  static constexpr void init_cmds(Cli &cli, std::size_t &index,
                                  CommandNode &parent, Cmd &cmd) {
    // initialize the node
    CommandNode &node = cli.cmds_[index];
    node.name = Cmd::name;
    node.description = Cmd::description;
    node.type = Cmd::type;
    // add the node to the parent
    parent.add_sub(node);
    // initialize sub commands of cmd
    for_each([&cli, &index,
              &node](Command auto &c) { init_cmds(cli, ++index, node, c); },
             cmd);
  }

  /**
   * @brief
   *
   * @param c
   * @return
   */
  Error putc(std::uint8_t c) {
    return rx_buf.push_back(c) ? Error::none : Error::buffer_overflow;
  }

  static bool is_end_of_message(char c);
  Error process_one(std::uint8_t c) {
    cli::ByteView esc_sequence{};
    switch (state_) {
    case idle:
      if (c == ansi::ESC or c == ansi::CSI) {
        state_ = ctrl_seq;
        esc_seq_index = current_line_.size() - 1;
        return Error::none;
      }
      return Error::none;
    case active:
      if (c == ansi::ESC) {
        // a control sequence has started
        state_ = ctrl_seq;
        current_ctrl_seq_.reset();
        esc_seq_index = current_line_.size() - 1;
        return Error::none;
      } else if (c == ansi::CSI) {
        // a control sequence has started
        state_ = ctrl_seq_param;
        esc_seq_index = current_line_.size() - 1;
        current_ctrl_seq_.reset();
        current_ctrl_seq_.introducer = {&current_line_[esc_seq_index], 1};
        return Error::none;
      } else if (ByteView(current_line_.data(), current_line_.size())
                     .ends_with(Cfg::command_terminator)) {
      } else {
        return Error::none;
      }
    case ctrl_seq: {
      if (c != '[') {
        // not a control sequence
        state_ = active;
        esc_seq_index = 0;
        return Error::none;
      }
      state_ = ctrl_seq_param;
      current_ctrl_seq_.introducer = {&current_line_[esc_seq_index], 2};
      return Error::none;

      const std::uint32_t esc_size = current_line_.size() - esc_seq_index;
      switch (esc_size) {
      case 2:
      case 3:
        switch (c) {
        case 'C':
          return process_esc_sequence(Control::cursor_right);
        case 'D':
          return process_esc_sequence(Control::cursor_left);
        case 's':
          return process_esc_sequence(Control::save_cursor);
        case 'u':
          return process_esc_sequence(Control::restore_cursor);
        case '@':
          return process_esc_sequence(Control::insert_char);
        case 'P':
          return process_esc_sequence(Control::delete_char);
        default:
          // unrecognized escape sequence
          return Error::invalid_esc_seq;
        }

      default:
        assert(esc_size != 1);
        return Error::invalid_esc_seq;
      }
    }
    case ctrl_seq_param:
      if (c >= 0x30u and c <= 0x3Fu) {

      } else if (c >= 0x20u and c <= 0x2Fu) {

      } else if (c >= 0x40u and c <= 0x7Eu) {

      } else {
      }
    default:
      return Error::invalid_cli_state;
    }
    while (not rx_buf.is_empty()) {
      std::uint8_t c = 0;
      rx_buf.pop(c);
      /* assuming no control characters
      switch (c) {
      case ansi::BEL:
      case ansi::SS2:
      case ansi::SS3:
      case ansi::DCS:
      case ansi::VT:
      case ansi::ST:
      case ansi::OSC:
      case ansi::SOS:
      case ansi::PM:
      case ansi::APC:
      case ansi::BS:
      case ansi::HT:
        [[fallthrough]];
      case ansi::FF:
        return Error::none;
      case ansi::LF:
        break;
      case ansi::CR:
        break;
      case ansi::ESC:
        break;
      case ansi::DEL:
        break;
      case ansi::CSI:
        break;
      default:
        break;
      }
      */

      if (is_end_of_message(c)) {
        return process_message();
      } else {
        if (not current_line_.push_back(c))
          return Error::buffer_overflow;
        if (current_line_.size() == 1) {
        }
      }
    }
  }

  void print() { print(root()); }

  void print(const CommandNode &c, std::size_t indent = 0) {
    std::cout << std::string(2 * indent, ' ') << std::string(c.name) << "["
              << std::string(c.type) << "]" << std::string(c.description);
    if (c.subcommand == nullptr) {
      std::cout << '\n';
      return;
    } else {
      std::cout << ":\n";
    }
    ++indent;
    const auto *sub = c.subcommand;
    while (sub != nullptr) {
      print(*sub, indent);
      sub = sub->next;
    }
  }

private:
  CommandNode *find_parent(CommandNode *root, VecView<ByteView>::iterator begin,
                           VecView<ByteView>::iterator end) {
    while (begin != end) {
      auto sub = root->subcommand;
      bool found = false;
      while (sub) {
        if (sub->name == *begin) {
          root = sub;
          found = true;
          break;
        }
        sub = sub->next;
      }
      if (not found)
        return nullptr;
      ++begin;
    }
    return root;
  }

  Error process_message() {
    // a whole message block is received, i.e. a complete command is
    // available.
    // 1. parse string and split it into the path and arguments
  }

  // the root is the "entry point" into the cli.
  constexpr CommandNode &root() noexcept { return cmds_[0]; }
  constexpr const CommandNode &root() const noexcept { return cmds_[0]; }

  Cfg config_;
  ControlSequence current_ctrl_seq_;
  // this points to the current command being typed in. Is nullptr if no
  // command is beind processed.
  CommandNode *current_cmd = nullptr;

  std::tuple<Commands...> commands_{};

  CommandNode cmds_[(num_cmds_v<Commands> + ...) + 1]{};
  // the buffers used for reception and transmit
  RingBuffer<uint8_t, Cfg::tx_size> tx_buf{};
  RingBuffer<uint8_t, Cfg::rx_size> rx_buf{};

  FixedSizeVector<uint8_t, Cfg::max_line_length> current_line_{};
  State state_ = State::idle;
  std::size_t esc_seq_index = 0;
  // the history and current command buffer
  // HistoryView &history_;

  // TransmitFunction transmit;
};

template <Config Cfg, Command... Commands>
Cli(Cfg &&, Commands &&...)
    -> Cli<std::remove_cvref_t<Cfg>, std::remove_cvref_t<Commands>...>;

template <typename Config, typename... Commands> class ACli {
  uint8_t rx_buf[Config::rx_size];
  uint8_t tx_buf[Config::tx_size];
};

template <Config Cfg, Command... Commands>
constexpr auto cli(Cfg &&config, Commands &&...commands) {
  return Cli<std::remove_cvref_t<Cfg>, std::remove_cvref_t<Commands>...>{
      std::forward<Cfg>(config), std::forward<Commands>(commands)...};
}
} // namespace cli
#endif
