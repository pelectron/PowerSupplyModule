#ifndef CLI_COMMAND_HPP
#define CLI_COMMAND_HPP
#include "cli/enums.hpp"
#include "cli/string.hpp"
#include "cli/type_list.hpp"

#include <span>
#include <type_traits>

namespace cli {

template <class T>
concept Name = std::convertible_to<std::decay_t<T>, ByteView> and
               not std::is_pointer_v<std::decay_t<T>>;

template <class C>
concept Command = requires(std::remove_cvref_t<C> &c, ExecType type,
                           const ByteView &args, std::span<uint8_t> &out) {
  { typename std::remove_cvref_t<C>::sub_command_list{} };
  { std::remove_cvref_t<C>::name } -> Name;
  { std::remove_cvref_t<C>::description } -> Name;
  { c.execute(type, args, out) } -> std::same_as<Error>;
};

struct CommandNode {
  /// holds the command's name
  ByteView name{};
  /// the command's description
  ByteView description{};
  /// the commands type as a string, i.e. the value type for parameters and the
  /// function signatures for functions.
  ByteView type{};
  void *this_ = nullptr;
  Error (*exec_)(void *, ExecType, ByteView, std::span<uint8_t> &) = nullptr;
  /// the next sibling command
  CommandNode *next = nullptr;
  /// pointers to the firstand last sub command of this
  CommandNode *subcommand = nullptr;
  CommandNode *last_subcommand = nullptr;

  class iterator {
    friend struct CommandNode;
    CommandNode *node = nullptr;
    constexpr iterator(CommandNode *node) noexcept : node(node) {}

  public:
    constexpr iterator(const iterator &) noexcept = default;
    constexpr iterator(iterator &&) noexcept = default;
    constexpr iterator &operator=(const iterator &) noexcept = default;
    constexpr iterator &operator=(iterator &&) noexcept = default;

    constexpr iterator &operator++() {
      node = node->next;
      return *this;
    }

    constexpr iterator operator++(int) {
      iterator ret{node};
      node = node->next;
      return ret;
    }

    constexpr CommandNode *operator->() { return node; }

    constexpr const CommandNode *operator->() const { return node; }

    constexpr CommandNode &operator*() { return *node; }

    constexpr const CommandNode &operator*() const { return *node; }

    constexpr auto operator<=>(const iterator &) const noexcept = default;
  };
  constexpr iterator begin() { return subcommand; }
  constexpr iterator end() const { return nullptr; }
  constexpr const iterator begin() const { return subcommand; }
  Error execute(ExecType exec_type, ByteView args, std::span<uint8_t> &out) {
    if (this_ and exec_)
      return (*exec_)(this_, exec_type, args, out);
    return Error::invalid_cmd;
  }

  constexpr void add_sub(CommandNode &c) {
    c.next = nullptr;
    c.subcommand = nullptr;
    c.last_subcommand = nullptr;
    CommandNode *sub = subcommand;
    if (sub == nullptr) {
      // empty
      subcommand = &c;
      last_subcommand = &c;
    } else if (sub->name > c.name) {
      // c should be inserted as first
      subcommand = &c;
      c.next = sub;
    } else if (last_subcommand->name < c.name) {
      // c should be inserted as last
      last_subcommand->next = &c;
      last_subcommand = &c;
    } else {
      // c should be inserted somewhere in the middle
      CommandNode *last_sub = sub;
      sub = sub->next;
      while (sub != nullptr) {
        if (sub->name > c.name) {
          last_sub->next = &c;
          c.next = sub;
          break;
        } else {
          last_sub = sub;
          sub = sub->next;
        }
      }
    }
  }
};

template <class Derived, Name CmdName, Name Description, Name Type,
          Command... SubCommands>
class CommandBase {
public:
  using sub_command_list = TypeList<SubCommands...>;
  using name_type = CmdName;
  static constexpr CmdName name{};
  static constexpr Description description{};
  static constexpr Type type{};

  CommandBase() = delete;
  constexpr CommandBase(const CommandBase &) = default;
  constexpr CommandBase(CommandBase &&) = default;
  constexpr CommandBase &operator=(const CommandBase &) = default;
  constexpr CommandBase &operator=(CommandBase &&) = default;

  template <Command... SubCommands_>
  constexpr CommandBase(SubCommands_ &&...cmds)
      : subcommands{std::forward<SubCommands_>(cmds)...} {}

  constexpr CommandBase(std::tuple<SubCommands...> &&cmds)
      : subcommands{std::move(cmds)} {}

  constexpr CommandBase(const std::tuple<SubCommands...> &cmds)
      : subcommands{cmds} {}

  constexpr Error execute(ExecType type, ByteView args,
                          std::span<uint8_t> &out) {
    return static_cast<Derived *>(this)->execute(type, args, out);
  }

protected:
  template <class D, Name C, Name Desc, Name H, Command... SubC>
  constexpr auto count_cmds(const CommandBase<D, C, Desc, H, SubC...> &c);
  template <class F, class D, Name C, Name Desc, Name H, Command... SubC,
            class... Args>
  friend constexpr void for_each(F &&f, CommandBase<D, C, Desc, H, SubC...> &t,
                                 Args &&...args);
  template <class F, class D, Name C, Name Desc, Name H, Command... SubC,
            class... Args>
  friend constexpr void
  for_each(F &&f, const CommandBase<D, C, Desc, H, SubC...> &t, Args &&...args);

  std::tuple<SubCommands...> subcommands{};
};

template <class Derived, Name CmdName, Name Description, Name Type>
class CommandBase<Derived, CmdName, Description, Type> {
public:
  using sub_command_list = TypeList<>;
  static constexpr CmdName name{};
  static constexpr Description description{};
  static constexpr Type type{};

  constexpr CommandBase() = default;
  constexpr CommandBase(const CommandBase &) = default;
  constexpr CommandBase(CommandBase &&) = default;
  constexpr CommandBase &operator=(const CommandBase &) = default;
  constexpr CommandBase &operator=(CommandBase &&) = default;

  constexpr Error execute(ExecType type, ByteView args,
                          std::span<uint8_t> &out) {
    return static_cast<Derived *>(this)->execute(type, args, out);
  }
};

} // namespace cli
#endif
