#ifndef CLI_CTTI_HPP
#define CLI_CTTI_HPP
#include "cli/traits.hpp"
#include "cli/util.hpp"

#include <array>
#include <source_location>
#include <string_view>
#include <type_traits>
#include <utility>

// clang-format off
#if defined(__clang__) && !defined(_MSC_VER)
#define CTTI_TYPE_PRETTY_FUNCTION_PREFIX                                       \
  "std::string_view cli::ctti::dtl::name() [T = "
#define CTTI_TYPE_PRETTY_FUNCTION_SUFFIX "]"
#elif defined(__GNUC__) && !defined(__clang__)
#define CTTI_TYPE_PRETTY_FUNCTION_PREFIX "std::string_view cli::ctti::dtl::name() [T = "
#define CTTI_TYPE_PRETTY_FUNCTION_SUFFIX "]"
#elif defined(_MSC_VER)
#if defined(__clang__)
#define CTTI_TYPE_PRETTY_FUNCTION_PREFIX "std::string_view __cdecl cli::ctti::dtl::name(void) [T = "
#define CTTI_TYPE_PRETTY_FUNCTION_SUFFIX "]"
#else
#define CTTI_TYPE_PRETTY_FUNCTION_PREFIX "class std::basic_string_view<char,struct std::char_traits<char> > __cdecl cli::ctti::dtl::name<"
#define CTTI_TYPE_PRETTY_FUNCTION_SUFFIX ">(void)"
#endif
#else
#error "No support for this compiler."
#endif
// clang-format on

#define CTTI_TYPE_PRETTY_FUNCTION_LEFT                                         \
  (sizeof(CTTI_TYPE_PRETTY_FUNCTION_PREFIX) - 1)
#define CTTI_TYPE_PRETTY_FUNCTION_RIGHT                                        \
  (sizeof(CTTI_TYPE_PRETTY_FUNCTION_SUFFIX) - 1)

namespace cli::ctti {

template <Name N, class T> struct Field {
  using name = N;
  using type = T;
  T value;
};

namespace dtl {
template <typename T> consteval std::string_view name_impl() {
  constexpr std::string_view name{
      std::source_location::current().function_name()};
#if defined(__clang__) and not defined(_MSC_VER)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(" ") + 1);
#elif defined(__GNUC__)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(" ") + 1);
#elif defined(_MSC_VER)
#if defined(__clang__)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(" ") + 1);
#else
  const auto split = name.substr(0, name.find_last_of(">"));
  const auto pos = split.find("impl<");
  const auto split2 = split.substr(pos + 5);
  if (const auto pos = split2.find(" "); pos != std::string_view::npos)
    return split2.substr(pos + 1);
  else
    return split2;
#endif
#endif
}

template <typename T> consteval auto name() {
  return []<std::size_t... Is>(std::index_sequence<Is...>) {
    return string_constant<name_impl<T>().data()[Is]...>{};
  }(std::make_index_sequence<name_impl<T>().size()>());
}

struct any_type {
  template <class T> constexpr operator T();
};

template <class TPtr> struct ptr {
  const TPtr *ptr;
  using type = TPtr;
};

template <class T> extern const T external;

template <auto Ptr>
[[nodiscard]] consteval auto member_name_impl() -> std::string_view {
  const auto name =
      std::string_view{std::source_location::current().function_name()};
#if defined(__clang__) and not defined(_MSC_VER)
  const auto split = name.substr(0, name.find("}]") - 1);
  return split.substr(split.find_last_of(".") + 1);
#elif defined(__GNUC__)
  const auto split = name.substr(0, name.find(")}"));
  return split.substr(split.find_last_of(":") + 1);
#elif defined(_MSC_VER)
#if defined(__clang__)
  const auto split = name.substr(0, name.find("}"));
  return split.substr(split.find_last_of(".") + 1);
#else
  const auto split = name.substr(0, name.find("}>"));
  return split.substr(split.find("->") + 2);
#endif
#endif
}

template <auto Value>
[[nodiscard]] consteval auto value_name_impl() -> std::string_view {
  const auto name =
      std::string_view{std::source_location::current().function_name()};
#if defined(__clang__) and not defined(_MSC_VER)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(": ") + 1);
#elif defined(__GNUC__)
  const auto split = name.substr(0, name.find_last_of(";"));
  return split.substr(split.find_last_of(": ") + 1);
#elif defined(_MSC_VER)
#if defined(__clang__)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(": ") + 1);
#else
  const auto split = name.substr(0, name.find_last_of(">"));
  const auto split2 = split.substr(split.find_last_of(": <") + 1);
  return split2.substr(0, split2.find_first_of("("));
#endif
#endif
}

template <auto &Value>
[[nodiscard]] consteval auto object_name_impl() -> std::string_view {
  const auto name =
      std::string_view{std::source_location::current().function_name()};
#if defined(__clang__) and not defined(_MSC_VER)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(": ") + 1);
#elif defined(__GNUC__)
  const auto split = name.substr(0, name.find_last_of(";"));
  return split.substr(split.find_last_of(": ") + 1);
#elif defined(_MSC_VER)
#if defined(__clang__)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(": ") + 1);
#else
  const auto split = name.substr(0, name.find_last_of(">"));
  const auto split2 = split.substr(split.find_last_of(": <") + 1);
  return split2.substr(0, split2.find_first_of("("));
#endif
#endif
}
template <auto Value>
[[nodiscard]] consteval auto enum_name_impl() -> std::string_view {
  const auto name =
      std::string_view{std::source_location::current().function_name()};
  return name;
#if defined(__clang__) and not defined(_MSC_VER)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(": ") + 1);
#elif defined(__GNUC__)
  const auto split = name.substr(0, name.find_last_of(";"));
  return split.substr(split.find_last_of(": ") + 1);
#elif defined(_MSC_VER)
#if defined(__clang__)
  const auto split = name.substr(0, name.find_last_of("]"));
  return split.substr(split.find_last_of(": ") + 1);
#else
  const auto split = name.substr(0, name.find_last_of(">"));
  const auto split2 = split.substr(split.find_last_of(": <") + 1);
  return split2.substr(0, split2.find_first_of("("));
#endif
#endif
}
template <auto V> consteval auto value_name() {
  return []<std::size_t... Is>(std::index_sequence<Is...>) {
    return string_constant<value_name_impl<V>().data()[Is]...>{};
  }(std::make_index_sequence<value_name_impl<V>().size()>());
}

template <auto &V> consteval auto object_name() {
  return []<std::size_t... Is>(std::index_sequence<Is...>) {
    return string_constant<object_name_impl<V>().data()[Is]...>{};
  }(std::make_index_sequence<object_name_impl<V>().size()>());
}

constexpr bool is_identifier(std::string_view sv) {
  if (sv.size() == 0)
    return false;
  if (not((sv[0] >= 'a' and sv[0] <= 'z') or (sv[0] >= 'A' and sv[0] <= 'Z') or
          sv[0] == '_')) {
    return false;
  }
  for (std::size_t i = 1; i < sv.size(); ++i) {
    if ((sv[0] >= 'a' and sv[0] <= 'z') or (sv[0] >= 'A' and sv[0] <= 'Z') or
        (sv[0] >= '0' and sv[0] <= '9') or sv[0] == '_')
      continue;
    else
      return false;
  }
  return true;
}

template <class E, std::size_t... Is>
consteval std::size_t generate_enum_names_size(std::index_sequence<Is...>) {
  std::array<std::pair<E, std::string_view>, sizeof...(Is)> strings{std::pair<
      E, std::string_view>{
      static_cast<E>(traits::enum_traits<E>::min + Is),
      value_name_impl<static_cast<E>(traits::enum_traits<E>::min + Is)>()}...};
  std::size_t size = 0;
  for (const auto &[e, s] : strings) {
    if (is_identifier(s)) {
      ++size;
    }
  }
  return size;
}

template <class E, std::size_t... Is>
consteval auto generate_enum_names(std::index_sequence<Is...>) {
  std::array<std::pair<E, std::string_view>, sizeof...(Is)> strings{std::pair<
      E, std::string_view>{
      static_cast<E>(traits::enum_traits<E>::min + Is),
      value_name_impl<static_cast<E>(traits::enum_traits<E>::min + Is)>()}...};
  std::array<std::pair<E, std::string_view>,
             generate_enum_names_size<E>(std::index_sequence<Is...>())>
      ret{};
  std::size_t size = 0;
  for (const auto &[e, s] : strings) {
    if (is_identifier(s)) {
      ret[size++] = {e, s};
    }
  }
  return ret;
}

template <class E, std::size_t... Is>
  requires traits::FlagEnum<E>
consteval std::size_t generate_enum_names_size(std::index_sequence<Is...>) {
  std::array<std::pair<E, std::string_view>, sizeof...(Is)> strings{
      std::pair<E, std::string_view>{
          static_cast<E>(1u << Is),
          value_name_impl<static_cast<E>(1u << Is)>()}...};
  std::size_t size = 0;
  for (const auto &[e, s] : strings) {
    if (is_identifier(s)) {
      ++size;
    }
  }
  return size;
}

template <class E, std::size_t... Is>
  requires traits::FlagEnum<E>
consteval auto generate_enum_names(std::index_sequence<Is...>) {
  std::array<std::pair<E, std::string_view>, sizeof...(Is)> strings{
      std::pair<E, std::string_view>{
          static_cast<E>(1u << Is),
          value_name_impl<static_cast<E>(1u << Is)>()}...};

  std::array<std::pair<E, std::string_view>,
             generate_enum_names_size<E>(std::index_sequence<Is...>())>
      ret{};

  std::size_t size = 0;
  for (const auto &[e, s] : strings) {
    if (is_identifier(s)) {
      ret[size++] = {e, s};
    }
  }
  return ret;
}

template <class E>
inline constexpr auto enum_name_map = generate_enum_names<E>(
    std::make_index_sequence<traits::enum_traits<E>::max -
                             traits::enum_traits<E>::min + 1>{});

template <class E>
  requires traits::FlagEnum<E>
inline constexpr auto enum_name_map<E> =
    generate_enum_names<E>(std::make_index_sequence<sizeof(E) * 8>{});

template <auto N> [[nodiscard]] consteval auto nth(auto... args) {
  return [&]<std::size_t... Ns>(std::index_sequence<Ns...>) {
    return [](decltype((void *)Ns)..., auto *nth, auto *...) {
      return *nth;
    }(&args...);
  }(std::make_index_sequence<N>{});
}

template <typename T> constexpr auto num_members() {
  if constexpr (requires {
                  T{any_type{}, any_type{}, any_type{}, any_type{}, any_type{},
                    any_type{}, any_type{}, any_type{}, any_type{}, any_type{},
                    any_type{}, any_type{}, any_type{}, any_type{}, any_type{},
                    any_type{}, any_type{}, any_type{}, any_type{}, any_type{}};
                }) {
    return 20;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}};
                       }) {
    return 19;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}};
                       }) {
    return 18;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}};
                       }) {
    return 17;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{}};
                       }) {
    return 16;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}};
                       }) {
    return 15;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}};
                       }) {
    return 14;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}};
                       }) {
    return 13;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{}};
                       }) {
    return 12;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}};
                       }) {
    return 11;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}};
                       }) {
    return 10;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}};
                       }) {
    return 9;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}, any_type{}};
                       }) {
    return 8;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}};
                       }) {
    return 7;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{},
                           any_type{}, any_type{}, any_type{}};
                       }) {
    return 6;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{},
                           any_type{}};
                       }) {
    return 5;
  } else if constexpr (requires {
                         T{any_type{}, any_type{}, any_type{}, any_type{}};
                       }) {
    return 4;
  } else if constexpr (requires { T{any_type{}, any_type{}, any_type{}}; }) {
    return 3;
  } else if constexpr (requires { T{any_type{}, any_type{}}; }) {
    return 2;
  } else if constexpr (requires { T{any_type{}}; }) {
    return 1;
  } else {
    return 0;
  }
}
template <auto N, class T> constexpr auto get(T &&t) {
  static_assert(num_members<std::remove_cvref_t<T>>() > 0,
                "T must have at least one and less than 20 members.");
  if constexpr (num_members<std::remove_cvref_t<T>>() == 20) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17, p18, p19, p20] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
    if constexpr (N == 13)
      return ptr<decltype(p14)>{&p14};
    if constexpr (N == 14)
      return ptr<decltype(p15)>{&p15};
    if constexpr (N == 15)
      return ptr<decltype(p16)>{&p16};
    if constexpr (N == 16)
      return ptr<decltype(p17)>{&p17};
    if constexpr (N == 17)
      return ptr<decltype(p18)>{&p18};
    if constexpr (N == 18)
      return ptr<decltype(p19)>{&p19};
    if constexpr (N == 19)
      return ptr<decltype(p20)>{&p20};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 19) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17, p18, p19] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
    if constexpr (N == 13)
      return ptr<decltype(p14)>{&p14};
    if constexpr (N == 14)
      return ptr<decltype(p15)>{&p15};
    if constexpr (N == 15)
      return ptr<decltype(p16)>{&p16};
    if constexpr (N == 16)
      return ptr<decltype(p17)>{&p17};
    if constexpr (N == 17)
      return ptr<decltype(p18)>{&p18};
    if constexpr (N == 18)
      return ptr<decltype(p19)>{&p19};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 18) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17, p18] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
    if constexpr (N == 13)
      return ptr<decltype(p14)>{&p14};
    if constexpr (N == 14)
      return ptr<decltype(p15)>{&p15};
    if constexpr (N == 15)
      return ptr<decltype(p16)>{&p16};
    if constexpr (N == 16)
      return ptr<decltype(p17)>{&p17};
    if constexpr (N == 17)
      return ptr<decltype(p18)>{&p18};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 17) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
    if constexpr (N == 13)
      return ptr<decltype(p14)>{&p14};
    if constexpr (N == 14)
      return ptr<decltype(p15)>{&p15};
    if constexpr (N == 15)
      return ptr<decltype(p16)>{&p16};
    if constexpr (N == 16)
      return ptr<decltype(p17)>{&p17};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 16) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
    if constexpr (N == 13)
      return ptr<decltype(p14)>{&p14};
    if constexpr (N == 14)
      return ptr<decltype(p15)>{&p15};
    if constexpr (N == 15)
      return ptr<decltype(p16)>{&p16};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 15) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15] =
        t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
    if constexpr (N == 13)
      return ptr<decltype(p14)>{&p14};
    if constexpr (N == 14)
      return ptr<decltype(p15)>{&p15};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 14) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
    if constexpr (N == 13)
      return ptr<decltype(p14)>{&p14};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 13) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
    if constexpr (N == 12)
      return ptr<decltype(p13)>{&p13};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 12) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
    if constexpr (N == 11)
      return ptr<decltype(p12)>{&p12};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 11) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
    if constexpr (N == 10)
      return ptr<decltype(p11)>{&p11};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 10) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
    if constexpr (N == 9)
      return ptr<decltype(p10)>{&p10};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 9) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
    if constexpr (N == 8)
      return ptr<decltype(p9)>{&p9};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 8) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
    if constexpr (N == 7)
      return ptr<decltype(p8)>{&p8};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 7) {
    auto &&[p1, p2, p3, p4, p5, p6, p7] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
    if constexpr (N == 6)
      return ptr<decltype(p7)>{&p7};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 6) {
    auto &&[p1, p2, p3, p4, p5, p6] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
    if constexpr (N == 5)
      return ptr<decltype(p6)>{&p6};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 5) {
    auto &&[p1, p2, p3, p4, p5] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
    if constexpr (N == 4)
      return ptr<decltype(p5)>{&p5};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 4) {
    auto &&[p1, p2, p3, p4] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
    if constexpr (N == 3)
      return ptr<decltype(p4)>{&p4};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 3) {
    auto &&[p1, p2, p3] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
    if constexpr (N == 2)
      return ptr<decltype(p3)>{&p3};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 2) {
    auto &&[p1, p2] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
    if constexpr (N == 1)
      return ptr<decltype(p2)>{&p2};
  } else if constexpr (num_members<std::remove_cvref_t<T>>() == 1) {
    auto &&[p1] = t;
    // structure bindings is not constexpr :/
    if constexpr (N == 0)
      return ptr<decltype(p1)>{&p1};
  }
}

template <class T, auto N> struct member_type {
  using type = std::remove_cvref_t<typename decltype(get<N>(
      external<std::remove_cvref_t<T>>))::type>;
};

template <class T, auto N> [[nodiscard]] consteval auto member_name() {
  constexpr auto name =
      member_name_impl<get<N>(external<std::remove_cvref_t<T>>)>();
  return [&]<auto... Ns>(std::index_sequence<Ns...>) {
    return string_constant<name[Ns]...>{};
  }(std::make_index_sequence<name.size()>{});
}

template <class T, auto N> using member_name_t = decltype(member_name<T, N>());

template <class T> [[nodiscard]] constexpr auto to_tuple(T &&t) {
  using namespace dtl;
  using T_ = std::remove_cvref_t<T>;
  if constexpr (num_members<T_>() == 20) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17, p18, p19, p20] = std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13},
        Field<member_name_t<T_, 13>, decltype(p14)>{.value = p14},
        Field<member_name_t<T_, 14>, decltype(p15)>{.value = p15},
        Field<member_name_t<T_, 15>, decltype(p16)>{.value = p16},
        Field<member_name_t<T_, 16>, decltype(p17)>{.value = p17},
        Field<member_name_t<T_, 17>, decltype(p18)>{.value = p18},
        Field<member_name_t<T_, 18>, decltype(p19)>{.value = p19},
        Field<member_name_t<T_, 19>, decltype(p20)>{.value = p20});
  } else if constexpr (num_members<T_>() == 19) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17, p18, p19] = std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13},
        Field<member_name_t<T_, 13>, decltype(p14)>{.value = p14},
        Field<member_name_t<T_, 14>, decltype(p15)>{.value = p15},
        Field<member_name_t<T_, 15>, decltype(p16)>{.value = p16},
        Field<member_name_t<T_, 16>, decltype(p17)>{.value = p17},
        Field<member_name_t<T_, 17>, decltype(p18)>{.value = p18},
        Field<member_name_t<T_, 18>, decltype(p19)>{.value = p19});
  } else if constexpr (num_members<T_>() == 18) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17, p18] = std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13},
        Field<member_name_t<T_, 13>, decltype(p14)>{.value = p14},
        Field<member_name_t<T_, 14>, decltype(p15)>{.value = p15},
        Field<member_name_t<T_, 15>, decltype(p16)>{.value = p16},
        Field<member_name_t<T_, 16>, decltype(p17)>{.value = p17},
        Field<member_name_t<T_, 17>, decltype(p18)>{.value = p18});
  } else if constexpr (num_members<T_>() == 17) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16, p17] = std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13},
        Field<member_name_t<T_, 13>, decltype(p14)>{.value = p14},
        Field<member_name_t<T_, 14>, decltype(p15)>{.value = p15},
        Field<member_name_t<T_, 15>, decltype(p16)>{.value = p16},
        Field<member_name_t<T_, 16>, decltype(p17)>{.value = p17});
  } else if constexpr (num_members<T_>() == 16) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15,
            p16] = std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13},
        Field<member_name_t<T_, 13>, decltype(p14)>{.value = p14},
        Field<member_name_t<T_, 14>, decltype(p15)>{.value = p15},
        Field<member_name_t<T_, 15>, decltype(p16)>{.value = p16});
  } else if constexpr (num_members<T_>() == 15) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15] =
        std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13},
        Field<member_name_t<T_, 13>, decltype(p14)>{.value = p14},
        Field<member_name_t<T_, 14>, decltype(p15)>{.value = p15});
  } else if constexpr (num_members<T_>() == 14) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14] =
        std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13},
        Field<member_name_t<T_, 13>, decltype(p14)>{.value = p14});
  } else if constexpr (num_members<T_>() == 13) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13] =
        std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12},
        Field<member_name_t<T_, 12>, decltype(p13)>{.value = p13});
  } else if constexpr (num_members<T_>() == 12) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12] =
        std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11},
        Field<member_name_t<T_, 11>, decltype(p12)>{.value = p12});
  } else if constexpr (num_members<T_>() == 11) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11] = std::forward<T>(t);
    return std::tuple(
        Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
        Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
        Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
        Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
        Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
        Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
        Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
        Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
        Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
        Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10},
        Field<member_name_t<T_, 10>, decltype(p11)>{.value = p11});
  } else if constexpr (num_members<T_>() == 10) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9, p10] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
                      Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
                      Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
                      Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
                      Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
                      Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
                      Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9},
                      Field<member_name_t<T_, 9>, decltype(p10)>{.value = p10});
  } else if constexpr (num_members<T_>() == 9) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8, p9] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
                      Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
                      Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
                      Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
                      Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
                      Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8},
                      Field<member_name_t<T_, 8>, decltype(p9)>{.value = p9});
  } else if constexpr (num_members<T_>() == 8) {
    auto &&[p1, p2, p3, p4, p5, p6, p7, p8] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
                      Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
                      Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
                      Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
                      Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7},
                      Field<member_name_t<T_, 7>, decltype(p8)>{.value = p8});
  } else if constexpr (num_members<T_>() == 7) {
    auto &&[p1, p2, p3, p4, p5, p6, p7] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
                      Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
                      Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
                      Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6},
                      Field<member_name_t<T_, 6>, decltype(p7)>{.value = p7});
  } else if constexpr (num_members<T_>() == 6) {
    auto &&[p1, p2, p3, p4, p5, p6] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
                      Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
                      Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5},
                      Field<member_name_t<T_, 5>, decltype(p6)>{.value = p6});
  } else if constexpr (num_members<T_>() == 5) {
    auto &&[p1, p2, p3, p4, p5] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
                      Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4},
                      Field<member_name_t<T_, 4>, decltype(p5)>{.value = p5});
  } else if constexpr (num_members<T_>() == 4) {
    auto &&[p1, p2, p3, p4] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3},
                      Field<member_name_t<T_, 3>, decltype(p4)>{.value = p4});
  } else if constexpr (num_members<T_>() == 3) {
    auto &&[p1, p2, p3] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2},
                      Field<member_name_t<T_, 2>, decltype(p3)>{.value = p3});
  } else if constexpr (num_members<T_>() == 2) {
    auto &&[p1, p2] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1},
                      Field<member_name_t<T_, 1>, decltype(p2)>{.value = p2});
  } else if constexpr (num_members<T_>() == 1) {
    auto &&[p1] = std::forward<T>(t);
    return std::tuple(Field<member_name_t<T_, 0>, decltype(p1)>{.value = p1});
  } else {
    static_assert(
        num_members<T_>() != 0,
        "Cannot convert T to a tuple because it either cannot be deconstructed "
        "as a structured binding, or because it has too many members.");
    return /*std::tuple()*/;
  }
}
} // namespace dtl

template <typename T> using name_t = decltype(::cli::ctti::dtl::name<T>());

template <typename T> consteval auto name() { return dtl::name<T>(); }

template <auto V> consteval auto value_name() { return dtl::value_name<V>(); }

template <auto &V> consteval auto object_name() {
  return dtl::object_name<V>();
}

template <traits::Enum E>
  requires(not traits::FlagEnum<E>)
constexpr std::string_view enum_name(E value) {
  for (const auto &[e, s] : dtl::enum_name_map<E>) {
    if (e == value)
      return s;
  }
  return "<"_sc + name<E>() + "::unknown>"_sc;
}

template <traits::Enum E>
  requires traits::FlagEnum<E>
constexpr std::size_t enum_name(E value) {
  for (const auto &[e, s] : dtl::enum_name_map<E>) {
    if ((e & value) == e)
      return s;
  }
  return "<"_sc + name<E>() + "::unknown>"_sc;
}

template <class T>
  requires std::is_aggregate_v<T>
struct StructInfo {
  /// a type list of FieldInfo :TypeList<FieldInfo...>
  using fields = decltype(dtl::to_tuple(std::declval<T>()));
};

template <class T> struct TypeInfo {
  using type = T;
  using name = cli::ctti::name_t<T>;
  using fields = TypeList<>;
  using field_tuple = std::tuple<>;
};

template <class T>
  requires std::is_aggregate_v<std::remove_cvref_t<T>>
struct TypeInfo<T> {
  using type = T;
  using name = cli::ctti::name_t<T>;
  /// a type list of FieldInfo :TypeList<FieldInfo...>
  using fields = decltype([]<std::size_t... Is>(std::index_sequence<Is...>) {
    using namespace dtl;
    return std::tuple<Field<decltype(member_name<T, Is>()),
                            typename member_type<T, Is>::type>...>{};
  }(std::make_index_sequence<dtl::num_members<T>()>()));
};

template <class T>
using field_tuple_t =
    decltype([]<std::size_t... Is>(std::index_sequence<Is...>) {
      using namespace dtl;
      return std::tuple<Field<decltype(member_name<T, Is>()),
                              typename member_type<T, Is>::type>...>{};
    }(std::make_index_sequence<dtl::num_members<T>()>()));

/**
 * @brief
 *
 * @tparam T
 * @param t
 * @return
 */
template <class T>
[[nodiscard]] constexpr auto to_tuple(T &&t) /* -> field_tuple_t<T> */ {
  return dtl::to_tuple(std::forward<T>(t));
}
/**
 * @brief
 *
 * @tparam T
 * @param tuple
 * @return
 */
template <class T>
[[nodiscard]] constexpr T from_tuple(const auto /* field_tuple_t<T> */ &tuple) {
  return [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    return T{std::get<Is>(tuple).value...};
  }(std::make_index_sequence<std::tuple_size_v<field_tuple_t<T>>>());
}
} // namespace cli::ctti
#endif
