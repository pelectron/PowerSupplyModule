#ifndef CLI_TRACKER_HPP
#define CLI_TRACKER_HPP
#include "cli/util.hpp"
#include <array>

namespace cli {

/**
 * @brief The Tracker is responsible for parsing the command name input into a
 * CommandNode. Optionally provides autocomplete functionality.
 * TODO: add differentiation between functions and params for autocomplete
 *
 * Thsi class only tracks command names, but not values (in case of params) or
 * arguments (in case of functions)
 * @tparam Depth the command tree depth
 * @tparam Depth the maximum length of any command name contained
 * @tparam AccessSeparator the character used to separate individual command
 * names
 * @tparam UseAutoComplete if true, autocomplete is enabled, else disabled
 */
template <std::size_t Depth, std::size_t MaxNameLength,
          char AccessSeparator = '.', bool UseAutoComplete = true>
class Tracker {
  CommandNode &root;
  std::array<CommandNode *, Depth> cmds{nullptr};
  smallest_type_for_value_t<Depth> size = 0;
  smallest_type_for_value_t<MaxNameLength + 1> cmd_size = 0;
  uint8_t last_char = 0;
  uint8_t buffer[MaxNameLength + 1]{};

  CommandNode *candidate() { return size < Depth ? cmds[size] : nullptr; }

public:
  /**
   * @brief create a tracker
   * @param root the command tree root node
   */
  constexpr Tracker(CommandNode &root) : root(root) {}

  /**
   * @brief needs to be called when a character is encountered
   *
   * @param c
   * @return
   */
  Error on_char(uint8_t c) {
    last_char = c;
    buffer[cmd_size++] = c;

    if (c == AccessSeparator) {
      // TODO: replace with better error
      if (size == Depth)
        return Error::buffer_overflow;
      if (cmd_size == 1)
        return Error::invalid_cmd;
      auto *cand = cmds[size];
      if (cand == nullptr)
        return Error::invalid_cmd;
      if (cand->name[cmd_size - 2] == buffer[cmd_size - 2]) {
        ++size;
        cmd_size = 0;
        return Error::none;
      } else {
        return Error::invalid_cmd;
      }
    }

    if (cmd_size == 1) {
      // set next candidate if there is one
      auto *parent = size == 0 ? &root : cmds[size - 1];
      for (auto &cmd : *parent) {
        if (cmd.name[0] == c) {
          cmds[size] = &cmd;
          return Error::none;
        }
      }
      return Error::invalid_cmd;
    }
    auto cand = candidate();
    const ByteView name((char *)buffer, cmd_size);
    if (cand == nullptr)
      return Error::invalid_cmd;

    if (cmd_size <= cand->name.size() and
        cand->name[cmd_size - 1] == buffer[cmd_size - 1]) {
      // keep candidate if it matched previously and continues to match
      return Error::none;
    }

    // need new candidate
    auto next = cand->next;
    if (next == nullptr) {
      return Error::invalid_cmd;
    }
    while (next != nullptr) {
      if (next->name.starts_with(name)) {
        cmds[size] = next;
        return Error::none;
      }
      next = next->next;
    }
    return Error::invalid_cmd;
  }
  /**
   * @brief needs to be called when a backspace is encountered
   */
  void on_backspace() {
    if (cmd_size == 0) {
      if (size == 0) {
        last_char = 0;
        return;
      }
      cmds[size] = nullptr;
      --size;
      for (const auto &ch : cmds[size]->name)
        buffer[cmd_size++] = ch;
    } else {
      --cmd_size;
      last_char = buffer[cmd_size - 1];
    }
  }

  ByteView on_autocomplete() {
    if constexpr (UseAutoComplete) {
      auto *const cand = candidate();
      if (cmd_size == 0 or cand == nullptr or last_char == AccessSeparator)
        return {};

      const ByteView name((char *)buffer, cmd_size);
      if (name == cand->name) {
        auto next = cand->next;
        if (next != nullptr and next->name.starts_with(name))
          return {};
        last_char = AccessSeparator;
        ++size;
        cmd_size = 0;
        return ".";
      }
      auto ret = cand->name.substr(cmd_size);
      for (const auto &ch : ret)
        buffer[cmd_size++] = ch;
      last_char = ret.back();
      return ret;
    } else {
      return {};
    }
  }

  CommandNode *cmd() {
    auto *ret = size < Depth ? cmds[size] : cmds[Depth - 1];
    if (cmd_size != ret->name.size())
      return nullptr;
    return ret;
  }

  void clear() {
    for (auto &cmd : cmds)
      cmd = nullptr;
    size = 0;
    cmd_size = 0;
    last_char = 0;
  }
};
} // namespace cli

#endif
