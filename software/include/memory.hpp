#ifndef PSM_MEMORY_HPP
#define PSM_MEMORY_HPP

#include <memory>
#include <type_traits>
#include <utility>

namespace psm {

template <typename T>
inline constexpr bool is_trivial_v =
    std::is_trivially_default_constructible_v<T> and
    std::is_trivially_destructible_v<T>;

#if __cplusplus >= 202002L
// wrapper around std:construct_at, needed when compiling with c++17
// in c++20, std::construct at is available and constexpr capable
template <typename T, typename... Args>
constexpr T *construct_at(T *t, Args &&...args) {
  return std::construct_at(t, std::forward<Args>(args)...);
}
#else
// in c++17, this function is only constexpr for trivially default con- and
// destructible types Else it falls back to placement new, which is never
// constexpr
template <typename T, typename... Args>
constexpr T *construct_at(T *t, Args &&...args) {
  if constexpr (is_trivial_v<T>) {
    *t = T(std::forward<Args>(args)...);
    return t;
  } else {
    return ::new (static_cast<void *>(t)) T(std::forward<Args>(args)...);
  }
}
#endif
} // namespace psm
#endif
