#ifndef CLI_UTIL_HPP
#define CLI_UTIL_HPP
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>

#include "cli/enums.hpp"
#include "cli/type_list.hpp"

namespace cli {
template <class> inline constexpr bool always_false = false;

using ByteView = std::string_view;

struct NoDescription {
  constexpr operator ByteView() const noexcept { return ""; }
};

struct NoHelp {
  constexpr operator ByteView() const noexcept { return ""; }
};

template <typename T, typename C>
concept StringOf = std::constructible_from<const C *, const C *> or
                   std::constructible_from<const C *, std::size_t>;

template <typename... T> using TypeList = type_list::TypeList<T...>;

template <char... c> struct string_constant {
  static constexpr char value[]{c..., 0};
  constexpr operator ByteView() const noexcept { return {value, sizeof...(c)}; }
  constexpr const char *data() const noexcept { return value; }
  constexpr std::size_t size() const noexcept { return sizeof...(c); }
};

template <char... C1, char... C2>
consteval bool operator==(const string_constant<C1...> &,
                          const string_constant<C2...> &) {
  return false;
}

template <char... Cs>
consteval bool operator==(const string_constant<Cs...> &,
                          const string_constant<Cs...> &) {
  return true;
}

template <char... C1, char... C2>
consteval bool operator!=(const string_constant<C1...> &,
                          const string_constant<C2...> &) {
  return true;
}

template <char... Cs>
consteval bool operator!=(const string_constant<Cs...> &,
                          const string_constant<Cs...> &) {
  return false;
}

template <std::size_t N> struct StringLiteral {
  char s[N]{0};
  constexpr StringLiteral(char const (&p)[N]) {
    for (std::size_t i = 0; i < N; ++i) {
      s[i] = p[i];
    }
  }

  template <char... C>
  constexpr StringLiteral(const auto... cs) : s{cs..., 0} {}

  template <char... Cs>
  constexpr StringLiteral(string_constant<Cs...>) : s{Cs..., 0} {}

  [[nodiscard]] constexpr auto
  operator<=>(const StringLiteral &) const = default;
  [[nodiscard]] constexpr operator ByteView() const { return {s, N - 1}; }
  [[nodiscard]] constexpr auto size() const -> std::size_t { return N - 1; }
};

template <std::size_t N> StringLiteral(char const (&p)[N]) -> StringLiteral<N>;

StringLiteral(const auto... cs) -> StringLiteral<sizeof...(cs) + 1>;

template <char... Cs>
StringLiteral(string_constant<Cs...>) -> StringLiteral<sizeof...(Cs) + 1>;

namespace dtl {
template <class StringConstant> struct to_lower;
template <char... Cs> struct to_lower<string_constant<Cs...>> {
  using type = string_constant<[](char c) {
    return c >= 'A' and c <= 'Z' ? c + ('a' - 'A') : c;
  }(Cs)...>;
};
} // namespace dtl
template <class T> using to_lower_t = typename dtl::to_lower<T>::type;
template <char... Cs>
constexpr to_lower_t<string_constant<Cs...>>
to_lower(const string_constant<Cs...> &) {
  return {};
}

template <char... C1, char... C2>
constexpr string_constant<C1..., C2...>
operator+(const string_constant<C1...> &, const string_constant<C2...> &) {
  return {};
}

template <char... C1, char... C2>
constexpr string_constant<C1..., C2...>
operator+(const string_constant<C1..., 0> &,
          const string_constant<C2..., 0> &) {
  return {};
}

template <char... C1, char... C2>
constexpr string_constant<C1..., C2...>
operator+(const string_constant<C1..., 0> &, const string_constant<C2...> &) {
  return {};
}

template <char... C1, char... C2>
constexpr string_constant<C1..., C2...>
operator+(const string_constant<C1...> &, const string_constant<C2..., 0> &) {
  return {};
}

namespace dtl {
template <StringLiteral S, std::size_t... Is>
static constexpr auto make_sc(std::index_sequence<Is...>) {
  return string_constant<S.s[Is]...>{};
}

template <typename F, class Tuple, class... Args, std::size_t... Is>
constexpr void for_each_impl(std::index_sequence<Is...>, F &&f, Tuple &&t,
                             Args &&...args) {
  (f(std::get<Is>(std::forward<Tuple>(t)), std::forward<Args>(args)...), ...);
}

template <class Tuple, class F, std::size_t... Is>
constexpr decltype(auto) apply_impl(Tuple &&t, F &&f,
                                    std::index_sequence<Is...>) {
  return std::forward<F>(f)(std::get<Is>(std::forward<Tuple>(t))...);
}
} // namespace dtl

template <StringLiteral S> constexpr auto operator""_sc() {
  return []<std::size_t... Is>(std::index_sequence<Is...>) {
    return string_constant<S.s[Is]...>{};
  }(std::make_index_sequence<S.size()>());
}

template <class Tuple, class F>
constexpr decltype(auto) apply(Tuple &&t, F &&f) {
  return dtl::apply_impl(std::forward<Tuple>(t), std::forward<F>(f),
                         std::make_index_sequence<
                             std::tuple_size_v<std::remove_cvref_t<Tuple>>>());
}

template <typename T> class RingBufView {
  T *arr_;
  std::size_t capacity_;
  // the head points to the position that was last written to
  T *head;
  // the tail points to the position that can be read from
  T *tail;
  std::size_t size_;

  constexpr T *incr(T *p) {
    ++p;
    if (p == (arr_ + capacity_))
      return arr_;
    else
      return p;
  }

public:
  constexpr RingBufView(T *arr, std::size_t capacity)
      : arr_(arr), capacity_(capacity), head(arr_), tail(arr_), size_(0) {}

  constexpr bool is_full() noexcept { return size_ == capacity_; }
  constexpr bool is_empty() noexcept { return size_ == 0; }

  constexpr std::size_t size() const { return size_; }
  constexpr std::size_t capacity() const { return capacity_; }

  constexpr bool push_back(const T &t) {
    auto s = size_;
    if (s == capacity_)
      return false;
    *head = t;
    size_ = s + 1;
    head = incr(head);
    return true;
  }

  constexpr bool push_back(T &&t) {
    auto s = size_;
    if (s == capacity_)
      return false;
    *head = std::move(t);
    size_ = s + 1;
    head = incr(head);
    return true;
  }

  constexpr void remove_last(std::size_t n) {
    auto s = size_;
    if (n > s)
      n = s;

    if (head - arr_ >= n) {
      head -= n;
    } else {
      n -= head - arr_;
      head = arr_ + capacity_ - n;
    }
  }

  constexpr bool pop(T &t) {
    auto s = size_;
    if (s == 0)
      return false;
    t = *tail;
    size_ = s - 1;
    tail = incr(tail);
    return true;
  }

  struct write_iterator {
    RingBufView *owner;
    T t{};
    constexpr T &operator*() { return t; }
    constexpr write_iterator &operator++() {
      owner->push_back(t);
      t = {};
      return *this;
    }
  };

  constexpr write_iterator output() { return {this, 0}; }
};

template <typename T> class VecView {
  T *values_;
  std::size_t size_;
  std::size_t capacity_;

public:
  using value_type = T;
  using iterator = T *;
  using const_iterator = const T *;

  constexpr VecView(T *arr, std::size_t capacity)
      : values_(arr), size_(0), capacity_(capacity) {}

  constexpr bool push_back(const T &t) {
    if (size_ == capacity_)
      return false;
    values_[size_++] = t;
    return true;
  }

  constexpr bool push_back(T &&t) {
    if (size_ == capacity_)
      return false;
    values_[size_++] = std::move(t);
    return true;
  }

  constexpr void remove_last(size_t n) {
    if (size_ < n)
      size_ = 0;
    else
      size_ -= n;
  }

  constexpr void reset() { size_ = 0; }

  constexpr std::size_t size() const { return size_; }
  constexpr std::size_t capacity() const { return capacity_; }

  constexpr T *data() { return values_; }
  constexpr const T *data() const { return values_; }

  constexpr T *begin() { return values_; }
  constexpr const T *begin() const { return values_; }

  constexpr T *end() { return values_ + size_; }
  constexpr const T *end() const { return values_ + size_; }

  constexpr T &operator[](std::size_t i) { return values_[i]; }
  constexpr const T &operator[](std::size_t i) const { return values_[i]; }
};

template <typename T, std::size_t Capacity>
class FixedSizeVector : public VecView<T> {
  static_assert(std::is_constructible_v<T>);
  static_assert(std::is_trivially_destructible_v<T>);
  T values_[Capacity]{};

public:
  constexpr FixedSizeVector() : VecView<T>(values_, Capacity) {}
};

template <typename T, std::size_t Capacity>
class RingBuffer : public RingBufView<T> {
  static_assert(std::is_constructible_v<T>);
  static_assert(std::is_trivially_destructible_v<T>);
  T values_[Capacity]{};

public:
  constexpr RingBuffer() : RingBufView<T>(values_, Capacity) {}
};

using ArgVector = VecView<ByteView>;

using OutputIterator = RingBufView<uint8_t>::write_iterator;

template <class T>
concept Name =
    std::convertible_to<T, ByteView> and not std::is_pointer_v<std::decay_t<T>>;

template <class C>
concept Command = requires(std::remove_cvref_t<C> &c, ExecType type,
                           const ArgVector &args, std::span<uint8_t> &out) {
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
  Error (*exec_)(void *, ExecType, const ArgVector &,
                 std::span<uint8_t> &) = nullptr;
  /// the next sibling command
  CommandNode *next = nullptr;
  /// pointers to the firstand last sub command of this
  CommandNode *subcommand = nullptr;
  CommandNode *last_subcommand = nullptr;

  Error execute(ExecType exec_type, ArgVector args, std::span<uint8_t> &out) {
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
          assert(sub->name != c.name);
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

  constexpr Error execute(ExecType type, ArgVector args,
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

  constexpr Error execute(ExecType type, ArgVector args,
                          std::span<uint8_t> &out) {
    return static_cast<Derived *>(this)->execute(type, args, out);
  }
};

template <class F, class Tuple, class... Args>
  requires(not Command<Tuple>)
constexpr void for_each(F &&f, Tuple &&t, Args &&...args) {
  if constexpr (Command<Tuple> and requires { t.subcommands; })
    dtl::for_each_impl(
        std::make_index_sequence<
            type_list::list_size_v<typename Tuple::sub_command_list>>(),
        std::forward<F>(f), t.subcommands, std::forward<Args>(args)...);
  else if constexpr (not Command<Tuple> and requires {
                       std::tuple_size_v<std::remove_cvref_t<Tuple>>;
                     })
    dtl::for_each_impl(std::make_index_sequence<
                           std::tuple_size_v<std::remove_cvref_t<Tuple>>>{},
                       std::forward<F>(f), std::forward<Tuple>(t),
                       std::forward<Args>(args)...);
}

template <class F, class D, Name C, Name Desc, Name H, Command... SubC,
          class... Args>
constexpr void for_each(F &&f, CommandBase<D, C, Desc, H, SubC...> &t,
                        Args &&...args) {
  if constexpr (requires { t.subcommands; })
    dtl::for_each_impl(std::make_index_sequence<sizeof...(SubC)>(),
                       std::forward<F>(f), t.subcommands,
                       std::forward<Args>(args)...);
}

template <class F, class D, Name C, Name Desc, Name H, Command... SubC,
          class... Args>
constexpr void for_each(F &&f, const CommandBase<D, C, Desc, H, SubC...> &t,
                        Args &&...args) {
  if constexpr (requires { t.subcommands; })
    dtl::for_each_impl(std::make_index_sequence<sizeof...(SubC)>(),
                       std::forward<F>(f), t.subcommands,
                       std::forward<Args>(args)...);
}

template <class T, class L> struct num_cmds;
template <class T, template <class...> class L, class... SubCmds>
struct num_cmds<T, L<SubCmds...>> {
  static constexpr std::size_t value =
      1 + (num_cmds<SubCmds, typename SubCmds::sub_command_list>::value + ...);
};
template <class T, template <class...> class L> struct num_cmds<T, L<>> {
  static constexpr std::size_t value = 1;
};

template <Command C>
inline constexpr std::size_t num_cmds_v =
    num_cmds<C, typename C::sub_command_list>::value;

template <class D, Name C, Name Desc, Name H, Command... SubC>
constexpr auto count_cmds(const CommandBase<D, C, Desc, H, SubC...> &c) {
  if constexpr (sizeof...(SubC) > 0) {
    1 + [&c]<std::size_t... Is>(std::index_sequence<Is...>) {
      return (count_cmds(std::get<Is>(c.subcommand)) + ...);
    }(std::make_index_sequence<sizeof...(SubC)>());
  } else
    return 1;
}

template <class D, Name C, Name Desc, Name H, Command... SubC>
constexpr auto count_level(const CommandBase<D, C, Desc, H, SubC...> &c) {
  if constexpr (sizeof...(SubC) > 0) {
    1 + [&c]<std::size_t... Is>(std::index_sequence<Is...>) {
      return std::max(count_level(std::get<Is>(c.subcommand))...);
    }(std::make_index_sequence<sizeof...(SubC)>());
  } else
    return 1;
}

/**
 * extracts the signature from a callable F, respecting const, volatile and
 * noexcept qualifications. The extracted signature can be accessed by its
 * inner typedef ``type``.
 *
 * F is either
 * - a signature type, i.e. ``Ret(Args...)[volatile][const][noexcept]``,
 * - a function pointer type, i.e. ``Ret(*)(Args...)[noexcept]``,
 * - a member function pointer type, i.e.
 * ``Ret(T::*)(Args...)[volatile][const][noexcept]``,
 * - or types with an unambigous call operator, i.e. a special case of the
 * member function pointer.
 *
 * for signature types, the signature itself is returned.
 * for function pointer types, the signature is formed by removing the pointer
 * for member function pointers and callables
 * @{
 */
template <typename F, typename = void> struct extract_signature;
template <typename R, typename... A> struct extract_signature<R(A...)> {
  using type = R(A...);
};
template <typename R, typename... A>
struct extract_signature<R(A...) noexcept> {
  using type = R(A...) noexcept;
};
template <typename R, typename... A> struct extract_signature<R(A...) const> {
  using type = R(A...) const;
};
template <typename R, typename... A>
struct extract_signature<R(A...) const noexcept> {
  using type = R(A...) const noexcept;
};
template <typename R, typename... A> struct extract_signature<R (*)(A...)> {
  using type = R(A...);
};
template <typename R, typename... A>
struct extract_signature<R (*)(A...) noexcept> {
  using type = R(A...) noexcept;
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...)> {
  using type = R(A...);
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) noexcept> {
  using type = R(A...) noexcept;
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) const> {
  using type = R(A...) const;
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) const noexcept> {
  using type = R(A...) const noexcept;
};
template <typename R, typename... A>
struct extract_signature<R(A...) volatile> {
  using type = R(A...) volatile;
};
template <typename R, typename... A>
struct extract_signature<R(A...) volatile noexcept> {
  using type = R(A...) volatile noexcept;
};
template <typename R, typename... A>
struct extract_signature<R(A...) volatile const> {
  using type = R(A...) volatile const;
};
template <typename R, typename... A>
struct extract_signature<R(A...) volatile const noexcept> {
  using type = R(A...) volatile const noexcept;
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile> {
  using type = R(A...) volatile;
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile noexcept> {
  using type = R(A...) volatile noexcept;
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile const> {
  using type = R(A...) volatile const;
};
template <typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile const noexcept> {
  using type = R(A...) volatile const noexcept;
};
template <typename F>
struct extract_signature<F, std::void_t<decltype(&F::operator())>> {
  using type = typename extract_signature<decltype(&F::operator())>::type;
};

template <typename F>
using extract_signature_t = typename extract_signature<F>::type;
/// @}

template <typename Signature> struct signature_traits;

template <typename R, class... A> struct signature_traits<R(A...)> {
  using signature_type = R(A...);
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...);
  using invoke_ptr_type = R (*)(void *, A...);
  using invoke_obj_type = void *;
  using mutable_obj_type = void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = false;
  template <typename F>
  static constexpr bool matches = std::is_invocable_r_v<R, F, A...>;
};
template <typename R, class... A> struct signature_traits<R(A...) noexcept> {
  using signature_type = R(A...) noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...) noexcept;
  using invoke_ptr_type = R (*)(void *, A...) noexcept;
  using invoke_obj_type = void *;
  using mutable_obj_type = void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = false;
  template <typename F>
  static constexpr bool matches = std::is_nothrow_invocable_r_v<R, F, A...>;
};
template <typename R, class... A> struct signature_traits<R(A...) const> {
  using signature_type = R(A...) const;
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...);
  using invoke_ptr_type = R (*)(const void *, A...);
  using invoke_obj_type = const void *;
  using mutable_obj_type = void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = false;
  template <typename F>
  static constexpr bool matches = std::is_invocable_r_v<R, const F, A...>;
};
template <typename R, class... A>
struct signature_traits<R(A...) const noexcept> {
  using signature_type = R(A...) const noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...) noexcept;
  using invoke_ptr_type = R (*)(const void *, A...) noexcept;
  using invoke_obj_type = const void *;
  using mutable_obj_type = void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = false;
  template <typename F>
  static constexpr bool matches =
      std::is_nothrow_invocable_r_v<R, const F, A...>;
};

template <typename R, class... A> struct signature_traits<R(A...) volatile> {
  using signature_type = R(A...) volatile;
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...);
  using invoke_ptr_type = R (*)(volatile void *, A...);
  using invoke_obj_type = volatile void *;
  using mutable_obj_type = volatile void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = true;
  template <typename F>
  static constexpr bool matches = std::is_invocable_r_v<R, volatile F, A...>;
};
template <typename R, class... A>
struct signature_traits<R(A...) volatile noexcept> {
  using signature_type = R(A...) volatile noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...) noexcept;
  using invoke_ptr_type = R (*)(volatile void *, A...) noexcept;
  using invoke_obj_type = volatile void *;
  using mutable_obj_type = volatile void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = true;
  template <typename F>
  static constexpr bool matches =
      std::is_nothrow_invocable_r_v<R, volatile F, A...>;
};
template <typename R, class... A>
struct signature_traits<R(A...) volatile const> {
  using signature_type = R(A...) const;
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...);
  using invoke_ptr_type = R (*)(volatile const void *, A...);
  using invoke_obj_type = volatile const void *;
  using mutable_obj_type = volatile void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = true;
  template <typename F>
  static constexpr bool matches =
      std::is_invocable_r_v<R, volatile const F, A...>;
};
template <typename R, class... A>
struct signature_traits<R(A...) volatile const noexcept> {
  using signature_type = R(A...) const noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (*)(A...) noexcept;
  using invoke_ptr_type = R (*)(volatile const void *, A...) noexcept;
  using invoke_obj_type = volatile const void *;
  using mutable_obj_type = volatile void *;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = true;
  template <typename F>
  static constexpr bool matches =
      std::is_nothrow_invocable_r_v<R, volatile const F, A...>;
};

template <typename F>
struct function_traits : signature_traits<extract_signature_t<F>> {};

template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...)> {
  using object_type = T;
  using signature_type = R(A...);
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...);
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = false;
};
template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...) noexcept> {
  using object_type = T;
  using signature_type = R(A...) noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...) noexcept;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = false;
};
template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...) const> {
  using object_type = T;
  using signature_type = R(A...) const;
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...) const;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = false;
};
template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...) const noexcept> {
  using object_type = T;
  using signature_type = R(A...) const noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...) const noexcept;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = false;
};

template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...) volatile> {
  using object_type = T;
  using signature_type = R(A...) volatile;
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...) volatile;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = true;
};
template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...) volatile noexcept> {
  using object_type = T;
  using signature_type = R(A...) volatile noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...) volatile noexcept;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = false;
  static constexpr bool is_volatile = true;
};
template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...) volatile const> {
  using object_type = T;
  using signature_type = R(A...) volatile const;
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...) volatile const;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = true;
};
template <typename R, class T, class... A>
struct function_traits<R (T::*)(A...) volatile const noexcept> {
  using object_type = T;
  using signature_type = R(A...) volatile const noexcept;
  using normalized_signature_type = R(A...);
  using ptr_type = R (T::*)(A...) volatile const noexcept;
  using return_type = R;
  using arguments = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const = true;
  static constexpr bool is_volatile = true;
};

template <class MemberPtr> struct member_traits;

template <class Type, class Obj> struct member_traits<Type Obj::*> {
  using type = Type;
  using object_type = Obj;
};
template <class Type, class Obj> struct member_traits<const Type Obj::*> {
  using type = const Type;
  using object_type = Obj;
};
template <class Type, class Obj> struct member_traits<volatile Type Obj::*> {
  using type = volatile Type;
  using object_type = Obj;
};
template <class Type, class Obj>
struct member_traits<const volatile Type Obj::*> {
  using type = const volatile Type;
  using object_type = Obj;
};

template <class MemberPtr>
using mem_data_type = typename member_traits<MemberPtr>::type;

template <class F, typename = void> struct is_callable : std::false_type {};

template <class F>
struct is_callable<F, std::void_t<function_traits<F>>> : std::true_type {};

template <class F> inline constexpr bool is_callable_v = is_callable<F>::value;

template <class F>
concept Callable = is_callable_v<F>;

template <class T, class MemFunPtr> struct MemFunBinder;

template <class T, typename Ret, typename... MemFunArgs>
struct MemFunBinder<T, Ret (T::*)(MemFunArgs...)> {
  T *t;
  Ret (T::*mem_fun)(MemFunArgs...);
  constexpr MemFunBinder(T &t, Ret (T::*mem_fun)(MemFunArgs...))
      : t(&t), mem_fun(mem_fun) {}
  constexpr MemFunBinder(const MemFunBinder &) = default;
  constexpr MemFunBinder(MemFunBinder &&) = default;
  constexpr MemFunBinder &operator=(const MemFunBinder &) = default;
  constexpr MemFunBinder &operator=(MemFunBinder &&) = default;

  Ret operator()(MemFunArgs... args) { return (t->*(mem_fun))(args...); }
};

template <class T, typename Ret, class... MemFunArgs>
struct MemFunBinder<T, Ret (T::*)(MemFunArgs...) noexcept> {
  T *t;
  Ret (T::*mem_fun)(MemFunArgs...) noexcept;

  constexpr MemFunBinder(T &t, Ret (T::*mem_fun)(MemFunArgs...) noexcept)
      : t(&t), mem_fun(mem_fun) {}
  constexpr MemFunBinder(const MemFunBinder &) = default;
  constexpr MemFunBinder(MemFunBinder &&) = default;
  constexpr MemFunBinder &operator=(const MemFunBinder &) = default;
  constexpr MemFunBinder &operator=(MemFunBinder &&) = default;

  Ret operator()(MemFunArgs... args) noexcept {
    return (t->*(mem_fun))(args...);
  }
};

template <class T, typename Ret, class... MemFunArgs>
struct MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const> {
  const T *t;
  Ret (T::*mem_fun)(MemFunArgs...) const;

  constexpr MemFunBinder(T &t, Ret (T::*mem_fun)(MemFunArgs...) const)
      : t(&t), mem_fun(mem_fun) {}
  constexpr MemFunBinder(const MemFunBinder &) = default;
  constexpr MemFunBinder(MemFunBinder &&) = default;
  constexpr MemFunBinder &operator=(const MemFunBinder &) = default;
  constexpr MemFunBinder &operator=(MemFunBinder &&) = default;
  Ret operator()(MemFunArgs... args) const { return (t.*(mem_fun))(args...); }
};

template <class T, typename Ret, class... MemFunArgs>
struct MemFunBinder<const T, Ret (T::*)(MemFunArgs...) const noexcept> {
  const T &t;
  Ret (T::*mem_fun)(MemFunArgs...) const noexcept;

  constexpr MemFunBinder(T &t, Ret (T::*mem_fun)(MemFunArgs...) const noexcept)
      : t(&t), mem_fun(mem_fun) {}
  constexpr MemFunBinder(const MemFunBinder &) = default;
  constexpr MemFunBinder(MemFunBinder &&) = default;
  constexpr MemFunBinder &operator=(const MemFunBinder &) = default;
  constexpr MemFunBinder &operator=(MemFunBinder &&) = default;
  Ret operator()(MemFunArgs... args) const noexcept {
    return (t->*(mem_fun))(args...);
  }
};

template <class T, class MemFunPtr>
MemFunBinder(T &&, MemFunPtr)
    -> MemFunBinder<std::remove_reference_t<T>, MemFunPtr>;

struct dummy {};
} // namespace cli
#endif
