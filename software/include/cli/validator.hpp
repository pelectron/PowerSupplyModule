#ifndef CLI_VALIDATE_HPP
#define CLI_VALIDATE_HPP
#include "cli/util.hpp"
#include "ctre.hpp"
#include <concepts>
#include <string_view>

namespace cli::validate {

template <Callable V> struct value_type {
  using type = std::remove_cvref_t<
      type_list::type_at_t<0, typename function_traits<V>::arguments>>;
};

template <Callable V> using value_type_t = typename value_type<V>::type;

template <class V>
concept Validator = requires(V &&v, const value_type_t<V> &value) {
  { v(value) } -> std::same_as<cli::Error>;
};

template <class V, class T>
concept ValidatorOf = requires(V &&v, const T &value) {
  { v(value) } -> std::same_as<cli::Error>;
};

template <class T> struct DefaultValidate {
  constexpr cli::Error operator()(const T &) const { return Error::none; }
};

template <StringLiteral S> struct regex {
  constexpr auto operator()(std::string_view subject) const noexcept
  /* -> ctre::regex_results */ {
    return ctre::match<S>(subject);
  }
};
} // namespace cli::validate
#endif
