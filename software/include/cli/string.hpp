#ifndef CLI_STRING_CONSTANT_HPP
#define CLI_STRING_CONSTANT_HPP
#include <cstddef>
#include <string_view>

namespace cli {

class ByteView : public std::string_view {
public:
  constexpr ByteView() = default;
  constexpr ByteView(const ByteView &) = default;
  constexpr ByteView(ByteView &&) = default;
  constexpr ByteView(std::string_view sv) : std::string_view(sv) {}
  constexpr ByteView &operator=(const ByteView &) = default;
  constexpr ByteView &operator=(ByteView &&) = default;
  using std::string_view::string_view;
  using std::string_view::operator=;
};

template <std::size_t N> struct StringLiteral;

template <char... c> struct string_constant {
  enum { string_size = sizeof...(c) };

  static constexpr char value[]{c..., 0};
  constexpr operator ByteView() const noexcept { return {value, sizeof...(c)}; }
  constexpr operator StringLiteral<sizeof...(c) + 1>() const noexcept {
    return {c...};
  }

  constexpr const char *data() const noexcept { return value; }
  constexpr std::size_t size() const noexcept { return sizeof...(c); }
};

template <char... C1, char... C2>
constexpr bool operator==(const string_constant<C1...> &,
                          const string_constant<C2...> &) {
  return false;
}

template <char... Cs>
constexpr bool operator==(const string_constant<Cs...> &,
                          const string_constant<Cs...> &) {
  return true;
}

template <char... C1, char... C2>
constexpr bool operator!=(const string_constant<C1...> &,
                          const string_constant<C2...> &) {
  return true;
}

template <char... Cs>
constexpr bool operator!=(const string_constant<Cs...> &,
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

  constexpr StringLiteral(ByteView str) {
    assert(str.size() < N);
    for (std::size_t i = 0; i < str.size(); ++i)
      s[i] = str[i];
  }

  template <class T> constexpr operator T() const {
    if constexpr (N == 1)
      return T();
    else
      return T{s, N - 1};
  }

  [[nodiscard]] constexpr auto
  operator<=>(const StringLiteral &) const = default;
  [[nodiscard]] constexpr operator ByteView() const { return {s, N - 1}; }
  [[nodiscard]] constexpr auto size() const -> std::size_t { return N - 1; }
  [[nodiscard]] constexpr const char *data() const noexcept { return s; }
  [[nodiscard]] constexpr char &operator[](std::size_t i) noexcept {
    return s[i];
  }
  [[nodiscard]] constexpr const char &operator[](std::size_t i) const noexcept {
    return s[i];
  }
  constexpr void clear() {
    for (auto &ch : s)
      ch = 0;
  }
};

template <std::size_t N> StringLiteral(char const (&p)[N]) -> StringLiteral<N>;

StringLiteral(const auto... cs) -> StringLiteral<sizeof...(cs) + 1>;

template <char... Cs>
StringLiteral(string_constant<Cs...>) -> StringLiteral<sizeof...(Cs) + 1>;

template <char... Cs> constexpr auto to_lower(const string_constant<Cs...> &) {
  return string_constant<[](char c) {
    return c >= 'A' and c <= 'Z' ? c + ('a' - 'A') : c;
  }(Cs)...>{};
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

template <StringLiteral S> constexpr auto operator""_sc() {
  return []<std::size_t... Is>(std::index_sequence<Is...>) {
    return string_constant<S.s[Is]...>{};
  }(std::make_index_sequence<S.size()>());
}

} // namespace cli
#endif
