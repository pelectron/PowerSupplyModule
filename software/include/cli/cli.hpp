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
#include "cli/ctti.hpp"
#include "cli/enums.hpp"
#include "cli/function.hpp"
#include "cli/help.hpp"
#include "cli/io.hpp"
#include "cli/param.hpp"
#include "cli/tracker.hpp"
#include "cli/util.hpp"

#include <array>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace cli {
using funcs::arg;
using funcs::operator""_arg;
using funcs::func;
using funcs::mem_fun;
using params::mem_data;
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

  Error execute(ExecType, const ArgVector &, std::span<char> &) {
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
  erase_char,
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
    {Control::insert_char, "\x1B[@"},  {Control::delete_char, "\x1B[1;P"},
    {Control::delete_char, "\x1B[1;X"}};

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
  static constexpr bool commands_start_with_separators = false;
  static constexpr std::size_t tx_size = 128;
  static constexpr std::size_t rx_size = 128;
  static constexpr bool use_autocomplete = true;
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

enum class State {
  active, // the cli is receiving a sequence of events that make up a
          // command name.
  args_start,
  set_params_start, // a param set
  call_params_start,
  call_params,
  set_params,
};

/**
 * @brief
 *
 * @tparam Cfg
 * @tparam Stream
 * @tparam Commands
 */
template <Config Cfg, io::OutputStream Stream, Command... Commands> class Cli {
public:
  using enum State;

  template <Config Cfg_, io::OutputStream S, Command... Cmds>
  constexpr Cli(Cfg_ &&cfg, S &&stream, Cmds &&...cmds)
      : config_{std::forward<Cfg_>(cfg)},
        commands_{std::forward<Cmds>(cmds)...}, out_(std::forward<S>(stream)) {
    init_tree();
  }

  template <Config Cfg_, io::BasicOutputStream S, Command... Cmds>
  constexpr Cli(Cfg_ &&cfg, S &&stream, Cmds &&...cmds)
      : config_{std::forward<Cfg_>(cfg)},
        commands_{std::forward<Cmds>(cmds)...},
        out_(io::AnsiOutputStream{std::forward<S>(stream)}) {
    init_tree();
  }

  constexpr Cli(Cli &&other)
      : config_{std::move(other.config_)},
        commands_{std::move(other.commands_)}, out_(std::move(other.out_)) {
    init_tree();
  }

  constexpr Cli(const Cli &other)
      : config_{other.config_}, commands_{other.commands_}, out_(other.out_) {
    init_tree();
  }

  constexpr Cli &operator=(Cli &&other) {
    config_ = std::move(other.config_);
    commands_ = std::move(other.commands_);
    out_ = std::move(other.out_);
    current_cmd = nullptr;
    tx_buf.reset();
    rx_buf.reset();
    current_line_.reset();
    state_ = active;
    tracker_.clear();
    esc_seq_index = 0;
    init_tree();
    return *this;
  }

  constexpr Cli &operator=(const Cli &other) {
    config_ = other.config_;
    commands_ = other.commands_;
    out_ = other.out_;
    current_cmd = nullptr;
    tx_buf.reset();
    rx_buf.reset();
    current_line_.reset();
    state_ = active;
    tracker_.clear();
    esc_seq_index = 0;
    init_tree();
    return *this;
  }

  Error put_event(const io::Event &ev) {
    if (not rx_buf.push_back(ev))
      return Error::buffer_overflow;
    return Error::none;
  }

  Error process() {
    while (1) {
      Error e = process_one();
      if (e == Error::none)
        continue;
      else if (e == Error::buffer_underflow)
        return Error::none;
      else
        return e;
    }
  }

  void print() { print(root(), 0); }

private:
  template <Command Cmd>
  static constexpr void init_cmd(Cli &cli, std::size_t &index,
                                 CommandNode &parent, Cmd &cmd) {
    // initialize the node
    CommandNode &node = cli.cmds_[index];
    node.name = Cmd::name;
    node.description = Cmd::description;
    node.type = Cmd::type;
    node.this_ = &cmd;
    node.exec_ = +[](void *this_, ExecType type, ArgVector args,
                     std::span<char> &out) -> Error {
      return static_cast<Cmd *>(this_)->execute(type, args, out);
    };
    // add the node to the parent
    parent.add_sub(node);
    // initialize sub commands of cmd
    for_each([&cli, &index,
              &node](Command auto &c) { init_cmd(cli, ++index, node, c); },
             cmd);
  }

  constexpr void init_tree() {
    // create the root
    CommandNode &root = cmds_[0];
    root.name = "root";
    root.description = "root";
    // add its sub commands
    std::size_t index = 1;
    init_cmd(*this, index, root, help);
    cli::for_each([&index, &root,
                   this](auto &cmd) { init_cmd(*this, ++index, root, cmd); },
                  commands_);
  }

  constexpr Error autocomplete() {
    const auto str = tracker_.on_autocomplete();
    out_.write(str);
    return Error::none;
  }

  constexpr Error write_char(uint8_t c) {
    if (Error err = out_.write(c); err != Error::none)
      return err;
    if (not current_line_.push_back(c))
      return Error::buffer_overflow;
    return Error::none;
  }

  Error process_char(uint8_t c) {
    switch (state_) {
    case active:
      switch (c) {
      case ' ':
        if constexpr (Cfg::access_separator == ' ') {
          if (auto *cmd = tracker_.cmd(); tracker_.candidate() == nullptr and
                                          cmd and cmd->subcommand == nullptr) {
            state_ = args_start;
          } else if (auto err = tracker_.on_char(c); err != Error::none) {
            return err;
          }
        } else {
          state_ = args_start;
        }
        return Error::none;
      case '(':
        if (auto err = out_.write(c); err != Error::none)
          return err;
        state_ = call_params_start;
        return Error::none;
      case '=':
        if (auto err = out_.write(c); err != Error::none)
          return err;
        state_ = set_params_start;
        return Error::none;
      default:
        if (auto err = tracker_.on_char(c); err != Error::none) {
          return err;
        } else {
          return out_.write(c);
        }
      }
    case args_start:
      switch (c) {
      case ' ':
        return Error::none;
      case '(':
        if (auto err = out_.write(c); err != Error::none)
          return err;
        state_ = call_params_start;
        return Error::none;
      case '=':
        if (auto err = out_.write(c); err != Error::none)
          return err;
        state_ = set_params_start;
        return Error::none;
      default:
        return Error::invalid_character;
      }
    case set_params_start:
      if (c != ' ') {
        if (auto err = write_char(c); err != Error::none)
          return err;
        state_ = set_params;
      }
      return Error::none;
    case call_params_start:
      if (c != ' ') {
        if (auto err = write_char(c); err != Error::none)
          return err;
        state_ = call_params;
      }
      return Error::none;
    case call_params:
      return write_char(c);
    case set_params:
      return write_char(c);
    default:
      return Error::invalid_state;
    }
    return Error::none;
  }

  Error backspace(uint8_t n = 1) {
    switch (state_) {
    case active:
      for (unsigned i = 0; i < n; ++i)
        tracker_.on_backspace();
      break;
    case args_start:
      state_ = active;
      for (unsigned i = 0; i < n; ++i)
        tracker_.on_backspace();
      break;
    case set_params_start:
      [[fallthrough]];
    case call_params_start:
      state_ = args_start;
      tracker_.on_backspace();
      --n;
      if (n != 0) {
        state_ = active;
        for (unsigned i = 0; i < n; ++i)
          tracker_.on_backspace();
      }
      break;
    case set_params:
      if (n == current_line_.size()) {
        state_ = set_params_start;
      }
      if (n > current_line_.size()) {
        state_ = active;
        auto rest = n - current_line_.size() - 1;
        for (unsigned i = 0; i < rest; ++i)
          tracker_.on_backspace();
      }
      current_line_.remove_last(n);
      break;
    case call_params:
      if (n == current_line_.size()) {
        state_ = call_params_start;
      }
      if (n > current_line_.size()) {
        state_ = active;
        auto rest = n - current_line_.size() - 1;
        for (unsigned i = 0; i < rest; ++i)
          tracker_.on_backspace();
      }
      current_line_.remove_last(n);
      break;
    default:
      return Error::invalid_state;
    }
    return out_.backspace(n);
  }

  Error process_one() {
    io::Event ev{};
    if (not rx_buf.pop(ev))
      return Error::buffer_underflow;

    switch (ev.type) {
    case io::Type::AutoComplete:
      return autocomplete();
    case io::Type::BackSpace:
      return backspace(ev.data[0]);
    case io::Type::Char:
      return process_char(ev.data[0]);
    case io::Type::CursorUp:
      return out_.cursor_up(ev.data[0]);
    case io::Type::CursorDown:
      return out_.cursor_down(ev.data[0]);
    case io::Type::CursorLeft:
      return out_.cursor_left(ev.data[0]);
    case io::Type::CursorRight:
      return out_.cursor_right(ev.data[0]);
    case io::Type::EraseInDisplay:
      return out_.erase_in_display(ev.data[0]);
    case io::Type::EraseInLine:
      return out_.erase_in_line(ev.data[0]);
    case io::Type::NewLine:
      return on_newline();
    case io::Type::ScrollUp:
      return out_.scroll_up(ev.data[0]);
    case io::Type::ScrollDown:
      return out_.scroll_down(ev.data[0]);
    default:
      // TODO: invalid event type error
      return Error::invalid_argument;
    }
  }

  void print(const CommandNode &c, std::size_t indent) {
    for (std::size_t i = 0; i < 2 * indent; ++i)
      out_.write(' ');
    out_.write(c.name);
    out_.write('[');
    out_.write(c.type);
    out_.write(']');
    out_.write(':');
    out_.write(c.description);
    if (c.subcommand == nullptr) {
      out_.newline();
      return;
    } else {
      out_.write(':');
      out_.newline();
    }
    ++indent;
    for (const auto &sub : c)
      print(sub, indent);
  }

  Error on_newline() {
    switch (state_) {
    case active:
      [[fallthrough]];
    case args_start:
      // a valid command without trailing "=value" is a get command
      return process_get_param();
    case set_params_start:
      // a value has not been entered
      return Error::expected_value;
    case set_params:
      // a valid command with trailing "= value" is a set command
      return process_set_param(
          ByteView(current_line_.data(), current_line_.size()));
    case call_params_start:
      // a parameters have not been entered
      return Error::expected_rparen;
    case call_params: {
      auto line = ByteView(current_line_.data(), current_line_.size());
      const auto closing_bracket_pos = line.find_last_of(')');
      if (closing_bracket_pos == ByteView::npos)
        return Error::expected_rparen;

      const auto last_char_pos = line.find_last_not_of(" )");
      if (closing_bracket_pos < last_char_pos and
          last_char_pos != ByteView::npos)
        return Error::unexpected_characters_after_closing_paren;

      return process_call(line.substr(0, last_char_pos + 1));
    }
    default:
      return Error::invalid_state;
    }
  }

  constexpr CommandNode &root() noexcept { return cmds_[0]; }
  constexpr const CommandNode &root() const noexcept { return cmds_[0]; }

  constexpr void return_to_idle() {
    // TODO: history
    current_line_.reset();
    start_of_args = nullptr;
    tracker_.clear();
    state_ = active;
  }
  constexpr Error return_to_idle(Error error) {
    return_to_idle();
    if (error != Error::none) {
      if (auto err = out_.write("Error: "); err != Error::none)
        return err;
      if (auto err = out_.write(ctti::enum_name(error)); err != Error::none)
        return err;
    }
    if (auto err = out_.newline(); err != Error::none)
      return err;
    return error;
  }
  constexpr Error process_get_param() {
    std::span<char> out{output_line_.data(), output_line_.size()};
    auto cmd = tracker_.cmd();

    if (auto err = out_.newline(); err != Error::none)
      return err;

    if (cmd == nullptr) {
      return return_to_idle(Error::invalid_cmd);
    }

    const auto error = cmd->execute(ExecType::get, {}, out);

    if (error != Error::none) {
      return return_to_idle(error);
    }

    if (auto err = out_.write(ByteView(out.data(), out.size()));
        err != Error::none)
      return err;

    if (auto err = out_.newline(); err != Error::none)
      return err;

    return_to_idle();
    return error;
  }

  constexpr Error process_set_param(ByteView args) {
    std::span<char> out{output_line_.data(), output_line_.size()};
    auto cmd = tracker_.cmd();

    if (auto err = out_.newline(); err != Error::none)
      return err;

    if (cmd == nullptr) {
      return return_to_idle(Error::invalid_cmd);
    }

    const auto error = cmd->execute(ExecType::set, args, out);

    return return_to_idle(error);
  }

  constexpr Error process_call(ByteView args) {
    std::span<char> out{output_line_.data(), output_line_.size()};
    auto cmd = tracker_.cmd();

    if (auto err = out_.newline(); err != Error::none)
      return err;

    if (cmd == nullptr) {
      return return_to_idle(Error::invalid_cmd);
    }

    const auto error = cmd->execute(ExecType::call, args, out);
    if (error != Error::none) {
      return return_to_idle(error);
    }

    if (auto err = out_.write(ByteView(out.data(), out.size()));
        err != Error::none)
      return err;

    if (auto err = out_.newline(); err != Error::none)
      return err;

    return_to_idle();
    return Error::none;
  }

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

  Error transmit_current_line() {
    auto err = out_.write(ByteView(Cfg::command_terminator));
    if (err != Error::none)
      return err;
    err = out_.write(ByteView(output_line_.begin(), output_line_.end()));
    if (err != Error::none)
      return err;
    return out_.write(ByteView(Cfg::command_terminator));
  }

  Error process_message() {
    // a whole message block is received, i.e. a complete command is
    // available.
    // 1. parse string and split it into the path and arguments
  }

  Cfg config_;
  // this points to the current command being typed in. Is nullptr if no
  // command is beind processed.
  CommandNode *current_cmd = nullptr;
  CommandNode *next_cmd = nullptr;

  std::tuple<Commands...> commands_{};
  template <class, class> friend struct Help;
  using HelpCmd = decltype(funcs::func(
      "help"_sc,
      cli::Help<Cli<Cfg, Stream, Commands...>, Cfg>{std::declval<Cli &>()},
      funcs::arg<ByteView, ""_sc>("cmd"_sc)));

  HelpCmd help = funcs::func(
      "help"_sc, cli::Help<Cli<Cfg, Stream, Commands...>, Cfg>{*this},
      funcs::arg<ByteView, ""_sc>("cmd"_sc));
  CommandNode cmds_[(num_cmds_v<Commands> + ...) + 2]{};
  // the buffers used for reception and transmit
  RingBuffer<uint8_t, Cfg::tx_size> tx_buf{};
  RingBuffer<io::Event, Cfg::rx_size> rx_buf{};

  FixedSizeVector<char, Cfg::max_line_length> current_line_{};
  std::array<char, Cfg::max_line_length> output_line_{};
  const char *start_of_args = nullptr;
  State state_ = active;
  std::size_t esc_seq_index = 0;
  // the history and current command buffer
  // HistoryView &history_;
  io::Output<Stream> out_;
  // the maximum depth of the command tree
  static constexpr std::size_t num_levels =
      std::max({num_levels_v<Commands>...}) + 1;
  // the maximum command length
  static constexpr std::size_t max_name_length =
      std::max({max_name_length_v<Commands>...});
  Tracker<num_levels, max_name_length, Cfg::access_separator,
          Cfg::use_autocomplete>
      tracker_{cmds_[0]};
  // TransmitFunction transmit;
};

template <Config Cfg, io::OutputStream Stream, Command... Commands>
Cli(Cfg &&, Stream &&, Commands &&...)
    -> Cli<std::remove_cvref_t<Cfg>, std::remove_cvref_t<Stream>,
           std::remove_cvref_t<Commands>...>;

template <Config Cfg, io::BasicOutputStream Stream, Command... Commands>
Cli(Cfg &&, Stream &&, Commands &&...)
    -> Cli<std::remove_cvref_t<Cfg>,
           io::AnsiOutputStream<std::remove_cvref_t<Stream>>,
           std::remove_cvref_t<Commands>...>;
} // namespace cli
#endif
