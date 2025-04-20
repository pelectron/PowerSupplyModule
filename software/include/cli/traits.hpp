#ifndef CLI_TRAITS_HPP
#define CLI_TRAITS_HPP

#include "cli/ctti.hpp"
#include <concepts>
#include <iterator>
#include <type_traits>

namespace cli::traits {
enum class Kind { Integer, FixPoint, Float, String, Struct, Sequence };

template <class T> struct kind;

/**
 * The basic categories of types the library can handle. These must be
 * overridden in order to opt in to the corresponding concept.
 * @{
 */
template <class T> struct is_integer : std::false_type {};
template <class T> struct is_float : std::false_type {};
template <class T> struct is_fixpoint : std::false_type {};
template <class T> struct is_string : std::false_type {};
template <class T> struct is_sequence : std::false_type {};
template <class T> struct is_struct : std::is_aggregate<T> {};
/**
 * @}
 */

template <class T>
concept Integer = std::integral<T> or is_integer<T>::value;

template <class T>
concept FixPoint = is_fixpoint<T>::value and
                   std::constructible_from<T, typename T::raw_value_type> and
                   std::integral<typename T::raw_value_type> and requires() {
                     { T::num_int_digits } -> std::convertible_to<std::size_t>;
                     { T::num_frac_digits } -> std::convertible_to<std::size_t>;
                   };

template <class T>
concept Float = std::floating_point<T> or is_float<T>::value;

template <class T>
concept String = is_string<T>::value and
                 std::constructible_from<T, const char *, std::size_t>;

template <class T>
concept Sequence = is_sequence<T>::value and requires(T a, const T b) {
  requires std::regular<T>;
  requires std::swappable<T>;
  requires std::destructible<typename T::value_type>;
  requires std::same_as<typename T::reference, typename T::value_type &>;
  requires std::same_as<typename T::const_reference,
                        const typename T::value_type &>;
  requires std::forward_iterator<typename T::iterator>;
  requires std::forward_iterator<typename T::const_iterator>;
  requires std::signed_integral<typename T::difference_type>;
  requires std::same_as<
      typename T::difference_type,
      typename std::iterator_traits<typename T::iterator>::difference_type>;
  requires std::same_as<typename T::difference_type,
                        typename std::iterator_traits<
                            typename T::const_iterator>::difference_type>;
  { a.begin() } -> std::same_as<typename T::iterator>;
  { a.end() } -> std::same_as<typename T::iterator>;
  { b.begin() } -> std::same_as<typename T::const_iterator>;
  { b.end() } -> std::same_as<typename T::const_iterator>;
  { a.cbegin() } -> std::same_as<typename T::const_iterator>;
  { a.cend() } -> std::same_as<typename T::const_iterator>;
  { a.size() } -> std::same_as<typename T::size_type>;
  { a.max_size() } -> std::same_as<typename T::size_type>;
  { a.empty() } -> std::same_as<bool>;
};

template <class T>
concept Struct = is_struct<T>::value and
                 requires(cli::ctti::field_tuple_t<T &&> tuple, T &&t) {
                   {
                     cli::ctti::to_tuple(t)
                   } -> std::convertible_to<cli::ctti::field_tuple_t<T>>;
                   { cli::ctti::from_tuple<T>(tuple) } -> std::same_as<T>;
                 };

template <std::integral T> struct integer_traits {
  using type = T;
  static constexpr bool is_signed = std::is_signed_v<type>;
  static constexpr auto size = sizeof(type);
  static constexpr auto aling = alignof(type);
};

template <std::floating_point T> struct float_traits {
  using type = T;
  static constexpr bool is_signed = std::is_signed_v<type>;
  static constexpr auto size = sizeof(type);
  static constexpr auto aling = alignof(type);
};

template <FixPoint T> struct fixpoint_traits {
  using type = T;
  using raw_value_type = typename T::raw_value_type;
  static constexpr bool is_signed = std::is_signed_v<raw_value_type>;
  static constexpr std::size_t num_int_digits = T::num_int_digits;
  static constexpr std::size_t num_frac_digits = T::num_frac_digits;
};

template <class T> struct string_traits : std::false_type {};

template <class T> struct sequence_traits : std::false_type {};

} // namespace cli::traits
#endif
