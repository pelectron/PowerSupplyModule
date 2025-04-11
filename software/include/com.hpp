#ifndef PSM_COM_HPP
#define PSM_COM_HPP

#include "cortex/function.hpp"
#include "error.hpp"
#include "fixed_map.hpp"
#include "parse.hpp"
#include "poly.hpp"
#include "poly/storage/ref_storage.hpp"
#include "poly/traits.hpp"
#include "psm.hpp"
#include "type_list/type_list.hpp"
#include <concepts>
#include <cstring>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>

namespace psm {
template <char... c> struct string_constant {};

template <char... cs> consteval auto operator""_sc() {
  return string_constant<cs...>{};
}

POLY_METHOD(putc);
POLY_METHOD(puts);

template <typename F, typename Ret, typename ArgList> struct top_level_adapter;
template <typename F, typename Ret, template <typename...> typename L,
          typename... Args>
struct top_level_adapter<F, Ret, L<Args...>> {
  Error operator()([[maybe_unused]] std::span<const char> str) {
    if constexpr (sizeof...(Args) == 0) {
      if constexpr (std::is_same_v<Ret, void>) {
        function();
        return {};
      } else if constexpr (std::is_same_v<Ret, Error>) {
        return function();
      } else {
        // TODO: static assert false
        function();
        return {};
      }
    } else {
      std::tuple<Args...> args;
      auto error = invoke<0>(str, args);
      if (error != Error::none) {
        return error;
      }
      if constexpr (std::is_convertible_v<Ret, Error>)
        return std::apply(function, args);
      else {
        std::apply(function, args);
        return {};
      }
    }

    return {};
  }

  F function;

private:
  template <size_t I>
  Error invoke(std::span<const char> str, std::tuple<Args...> &args) {
    auto res = fmt::parse<cm::type_at_t<I, type_list::TypeList<Args...>>>(str);

    if (not res.value.has_value()) {
      return res.value.error();
    }

    std::get<I>(args) = res.value.value();

    if constexpr (sizeof...(Args) == I + 1)
      return {};
    else
      return invoke<I + 1>(str.subspan(res.ptr - str.data()), args);
  }
};

template <poly::Storage S, typename Properties, typename Methods>
class ObjectAdapter;

template <poly::Storage S, template <typename...> typename L1,
          template <typename...> typename L2,
          poly::PropertySpecification... Properties,
          poly::MethodSpecification... Methods>
class ObjectAdapter<S, L1<Properties...>, L2<Methods...>> {
  Error operator()([[maybe_unused]] std::span<const char> str,
                   std::span<char> output) {
    size_t size = 0;
    for (const auto &ch : str) {
      if (ch == ' ' or ch == '\0')
        break;
      ++size;
    }

    auto name = str.subspan(0, size);

    if (name.size() == 0)
      return Error::invalid_param;

    auto rest = str.subspan(size + 1);

    if (((name == psm::method_name<poly::method_name_t<Methods>>::value) or
         ...)) {
      return invoke<0>(name, rest);
    } else if (memcmp(name.data(), "get", 3) == 0) {
      size_t size = 0;
      for (const auto &ch : rest) {
        if (ch == ' ' or ch == '\0')
          break;
        ++size;
      }
      auto param_name = rest.subspan(0, size);
      if (((param_name ==
            psm::property_name<poly::property_name_t<Properties>>::value) or
           ...)) {
        auto res = get_property<0>(param_name, output);
        if (not res.has_value())
          return res.error();
        transmit(res.value());
        return Error::none;
      } else {
        return Error::invalid_param;
      }
    } else if (memcmp(name.data(), "set", 3) == 0) {
      size_t size = 0;
      for (const auto &ch : rest) {
        if (ch == ' ' or ch == '\0')
          break;
        ++size;
      }
      auto param_name = rest.subspan(0, size);
      auto param_value = rest.subspan(size + 1);
      if (param_value.size() == 0)
        return Error::invalid_param;
      if (((param_name ==
            psm::property_name<poly::property_name_t<Properties>>::value) or
           ...)) {
        return set_property<0>(param_name, param_value);
      } else {
        return Error::invalid_param;
      }
    } else {
      return Error::invalid_param;
    }
  }

  template <size_t I = 0>
  Result<uint32_t> get_property(std::span<const char> name,
                                std::span<char> output) {
    using props = poly::type_list<Properties...>;
    using prop = type_list::type_at_t<I, props>;
    if constexpr (I == sizeof...(Properties)) {
      return Error::invalid_param;
    } else {

      if (memcmp(prop::name, name.data(), name.size()) == 0) {
        return fmt::format_to(output, obj.template get<prop>());
      } else {
        return get_property(name, output);
      }
    }
  }

  template <size_t I = 0>
  Error set_property(const std::span<const char> &name,
                     const std::span<const char> &value) {
    if constexpr (I == sizeof...(Properties)) {
      return Error::invalid_param;
    } else {
      using props = poly::type_list<Properties...>;
      using prop = type_list::type_at_t<I, props>;
      using value_type = poly::value_type_t<prop>;

      if (memcmp(prop::name, name.data(), name.size()) == 0) {
        auto res = fmt::parse<value_type>(value);
        if (not res.value.has_value()) {
          return res.error;
        }
        obj.template set<prop>(res.value.value());
        return Error::none;
      } else {
        return set_property<I + 1>(name, value);
      }
    }
  }

  template <size_t I>
  Error invoke(std::span<const char> method, std::span<const char> args) {
    // std::tuple<Args...> args;
    // auto res = fmt::parse<cm::type_at_t<0,
    // type_list::TypeList<Args...>>>(str);
    //
    // if (not res.value.has_value()) {
    //   return res.value.error();
    // }
    //
    // std::get<I>(args) = res.value.value();
    //
    // if constexpr (sizeof...(Args) == I + 1)
    //   return {};
    // else
    //   return invoke<I + 1>(str.subspan(res.ptr - str.data()), args);
  }

  void transmit(uint32_t size) {}

  poly::Struct<poly::ref_storage, L1<Properties...>, L2<Methods...>> obj;

  FixedMap<std::span<const char>,
           cm::MoveOnlyFunction<Error(std::span<const char>), 4, 4>,
           sizeof...(Methods)>
      methods;
  FixedMap<std::span<const char>,
           cm::MoveOnlyFunction<Error(std::span<const char>, void *), 4, 4>,
           sizeof...(Methods)>
      getters;
  FixedMap<std::span<const char>,
           cm::MoveOnlyFunction<Error(std::span<const char>), 4, 4>,
           sizeof...(Methods)>
      setters;
};

enum class Delimiter { none, r, n, rn };

struct cmd {

  template <std::invocable<const char *, std::size_t> F>
  cmd(std::span<const char> name, F &&function) {}

  std::span<const char> name;
  cm::MoveOnlyFunction<Error(const char *, std::size_t), 32, 4> exec;
};

template <typename F> struct cmd_exec {
  Error operator()(const char *, std::size_t) { return {}; }
  F function;
};

struct sub {};

class ComSlave {
public:
  void init() {
    add_cmd("get", [this](std::span<const char>) -> Error { return {}; });
    add_cmd("set", [this](std::span<const char>) -> Error { return {}; });
  }

  void set_rx_buffer(std::span<std::uint8_t>);
  void set_tx_buffer(std::span<std::uint8_t>);

  uint32_t put_and_get_tx_size(char c);
  uint32_t on_receive(const char *str, uint32_t size);

  Delimiter delimiter() const;

  // add a top level command that does not have any sub commands
  // top level command syntax: "name [args...]"
  // {

  template <std::invocable<std::span<const char>> F>
  void add_cmd(std::span<const char> name, F &&function) {
    commands.insert(name, std::forward<F>(function));
  }

  template <std::invocable F>
  void add_cmd(std::span<const char> name, F &&function) {
    commands.insert(
        name, [f = std::forward<F>(function)](std::span<const char>) -> Error {
          if constexpr (std::is_same_v<std::invoke_result_t<F>, void>) {
            f();
            return {};
          } else if constexpr (std::is_same_v<std::invoke_result_t<F>, Error>) {
            return f();
          } else {
            // TODO: static assert false
            f();
            return {};
          }
        });
  }

  template <typename F> void add_cmd(std::span<const char> name, F &&function) {
    using func = cm::function_traits<F>;
    using ret = typename func::return_type;
    using args = typename func::arguments;

    commands.insert(name, top_level_adapter<std::decay_t<F>, ret, args>(
                              std::forward<F>(function)));
  }
  // }

  template <poly::Storage S, poly::PropertySpecification... Properties,
            poly::MethodSpecification... Methods>
  void add_obj(const char *name,
               poly::Interface<S, poly::type_list<Properties...>,
                               poly::type_list<Methods...>>
                   obj);

private:
  using CmdFunc = cm::MoveOnlyFunction<Error(std::span<const char>), 24, 4>;
  FixedMap<std::span<const char>, CmdFunc, 128> commands{};
  FixedVec<std::span<const char>, 16> objects;
  struct Delegate {};
};

} // namespace psm

#endif
