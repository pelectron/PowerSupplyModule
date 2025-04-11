/*
 * result.hpp
 *
 *  Created on: Feb 27, 2024
 *      Author: pelec
 */

#ifndef RESULT_HPP_
#define RESULT_HPP_
#include "error.hpp"
#include "memory.hpp"

#include <cassert>

namespace psm {

namespace result_dtl {
class ResultBase {
public:
  constexpr ResultBase() noexcept = default;

  constexpr ResultBase(Error e) noexcept : error_(e) {}
  constexpr ResultBase(const ResultBase &) = default;
  constexpr ResultBase(ResultBase &&) = default;
  constexpr ResultBase &operator=(const ResultBase &) = default;
  constexpr ResultBase &operator=(ResultBase &&) = default;

  constexpr operator bool() const noexcept { return error_ == Error::none; }
  constexpr Error error() const noexcept { return error_; }
  constexpr bool has_value() const { return error_ == Error::none; }

protected:
  constexpr void set_error(Error e) noexcept { error_ = e; }

private:
  Error error_{Error::none};
};
struct Empty {};

template <typename T, bool trivial> class ResultImpl : public ResultBase {
public:
  constexpr ResultImpl() noexcept : ResultBase(Error::empty_result) {}
  constexpr ResultImpl(psm::Error e) noexcept : ResultBase(e) {}
  constexpr ResultImpl(T &&t) noexcept(std::is_nothrow_move_constructible_v<T>)
      : data_(std::move(t)) {}
  constexpr ResultImpl(const T &t) noexcept(
      std::is_nothrow_copy_constructible_v<T>)
      : data_(t) {}

  constexpr ResultImpl(const ResultImpl &other) noexcept(
      std::is_nothrow_copy_constructible_v<T>)
      : ResultBase(other.error()) {
    if (other.has_value())
      psm::construct_at(&data_.t, other.data_.t);
  }

  constexpr ResultImpl(ResultImpl &&other) noexcept(
      std::is_nothrow_move_constructible_v<T>)
      : ResultBase(other.error()),
        data_(other.has_value() ? Elem(std::move(other.value())) : Elem()) {
    if (other.has_value())
      psm::construct_at(&data_.t, other.data_.t);
  }

  constexpr ResultImpl &operator=(const ResultImpl &other) noexcept(
      std::is_nothrow_copy_assignable_v<T>) {
    reset();
    set_error(other.error());
    if (other) {
      data_.t = other.data_.t;
    }
    return *this;
  }
  constexpr ResultImpl &
  operator=(ResultImpl &&other) noexcept(std::is_nothrow_move_assignable_v<T>) {
    reset();
    set_error(other.error());
    if (other) {
      data_.t = std::move(other.data_.t);
    }
    return *this;
  }

  constexpr T &value() & { return data_.t; }
  constexpr const T &value() const & { return data_.t; }
  constexpr T &&value() && { return std::move(data_.t); }
  constexpr const T &&value() const && { return std::move(data_.t); }

private:
  constexpr void reset() {
    if (*this)
      std::destroy_at(&data_.t);
  }
  union Elem {
    constexpr Elem() : e() {}
    constexpr Elem(const T &x) { psm::construct_at(&t, x); }
    constexpr Elem(T &&x) { psm::construct_at(&t, std::move(x)); }

    template <typename U> constexpr Elem(U &&u) {
      psm::construct_at(&(this->t), std::forward<U>(u));
    }
    constexpr Elem(Elem &&) {}
    constexpr Elem(const Elem &) {}
    constexpr ~Elem() {}
    Empty e;
    T t;
  };
  Elem data_{};
};

template <typename T> class ResultImpl<T, true> : public ResultBase {
public:
  constexpr ResultImpl() noexcept : ResultBase(Error::empty_result) {}
  constexpr ResultImpl(psm::Error e) noexcept : ResultBase(e) {
    assert(e != Error::none);
  }
  constexpr ResultImpl(T &&t) noexcept(std::is_nothrow_move_constructible_v<T>)
      : data_(std::move(t)) {}
  constexpr ResultImpl(const T &t) noexcept(
      std::is_nothrow_copy_constructible_v<T>)
      : data_(t) {}

  constexpr ResultImpl(const ResultImpl &other) noexcept(
      std::is_nothrow_copy_constructible_v<T>)
      : ResultBase(other.error()) {
    if (other.has_value())
      data_.t = other.data_.t;
  }

  constexpr ResultImpl(ResultImpl &&other) noexcept(
      std::is_nothrow_move_constructible_v<T>)
      : ResultBase(other.error()) {
    if (other.has_value())
      data_.t = other.data_.t;
  }

  constexpr ResultImpl &operator=(const ResultImpl &other) noexcept(
      std::is_nothrow_copy_assignable_v<T>) {
    data_.e = Empty{};
    set_error(other.error());
    if (other) {
      data_.t = other.data_.t;
    }
    return *this;
  }
  constexpr ResultImpl &
  operator=(ResultImpl &&other) noexcept(std::is_nothrow_move_assignable_v<T>) {
    data_.e = Empty{};
    set_error(other.error());
    if (other) {
      data_.t = std::move(other.data_.t);
    }
    return *this;
  }

  constexpr T &value() & { return data_.t; }
  constexpr const T &value() const & { return data_.t; }
  constexpr T &&value() && { return std::move(data_.t); }
  constexpr const T &&value() const && { return std::move(data_.t); }

private:
  union Elem {
    constexpr Elem() : e() {}
    constexpr Elem(const T &t) : t(t) {}
    constexpr Elem(T &&t) : t(std::move(t)) {}
    constexpr Elem(Elem &&) {}
    constexpr Elem(const Elem &) {}
    Empty e;
    T t;
  };
  Elem data_{};
};
} // namespace result_dtl

/**
 * An optional-like type which either contains a T, or a psm::Error value.
 * Wether the Result contains a value can be queried with operator bool() or
 * has_value(). The value of the result, if it contains a value, can be
 * retrievied with value(). The error can be retrieved with error().
 *
 * If T is trivial, i.e. trivially default constructible and trivially
 * destructible, then Result is completely constexpr from c++17 onward. In c++20
 * and above, the whole class is constexpr, even for non trivial types.
 *
 * @tparam T the type of value to store
 */
template <typename T>
class Result : public result_dtl::ResultImpl<T, is_trivial_v<T>> {
  using Base = result_dtl::ResultImpl<T, is_trivial_v<T>>;

public:
  /// constructs an erroneous Result with Error::empty_result. If T is void,
  /// this will create a Result with Error::none.
  constexpr Result() noexcept = default;

  /// construct an erroneous Result with error e.
  /// e must not be Error::none.
  constexpr Result(psm::Error e) noexcept : Base(e) {
    assert(e != Error::none);
  }

  /// construct a Result with a t.
  /// @{
  constexpr Result(T &&t) noexcept(std::is_nothrow_move_constructible_v<T>)
      : Base(std::move(t)) {}
  constexpr Result(const T &t) noexcept(std::is_nothrow_copy_constructible_v<T>)
      : Base(t) {}
  /// @}

  // template <typename U, typename = std::enable_if_t<
  //                           not std::is_same_v<std::decay_t<U>, Result>>>
  // constexpr Result(U &&u) noexcept(std::is_nothrow_constructible_v<T, U &&>)
  //     : Base(std::forward<U>(u)) {}

  constexpr Result(const Result &other) noexcept(
      std::is_nothrow_copy_constructible_v<T>)
      : Base(other) {}

  constexpr Result(Result &&other) noexcept(
      std::is_nothrow_move_constructible_v<T>)
      : Base(other) {}

  constexpr Result &operator=(const Result &other) noexcept(
      std::is_nothrow_copy_assignable_v<T>) {
    Base::operator=(other);
    return *this;
  }

  constexpr Result &
  operator=(Result &&other) noexcept(std::is_nothrow_move_assignable_v<T>) {
    Base::operator=(std::move(other));
    return *this;
  }

  /// returns true if this contains a value
  constexpr operator bool() const noexcept { return Base::operator bool(); }

  /// retuns true if this contains a value
  constexpr bool has_value() const noexcept { return Base::has_value(); }

  /// get the error associated with the result. If this contains a value,
  /// Error::none is returned.
  constexpr Error error() const noexcept { return Base::error(); }

  /**
   * Access to value in this. Calling value() on an empty/erronoeus result
   * causes undefined behaviour.
   * @{
   */
  constexpr T &value() & { return Base::value(); }
  constexpr const T &value() const & { return Base::value(); }
  constexpr T &&value() && { return std::move(Base::value()); }
  constexpr const T &&value() const && { return std::move(Base::value()); }
  /// @}
};

/**
 * Specialization of Result for void. The value cannot be retrieved, only the
 * error.
 *
 * The main difference between Result<T> and Result<void> is the default
 * constructed state. A default constructed Result<T> is erroneous, i.e. it does
 * not have a value. A default constructed Result<void> "has" a value, i.e. it
 * is constructed with the error Error::none and operator bool() and has_value()
 * return true.
 */
template <> class Result<void> : public result_dtl::ResultBase {
  using Base = result_dtl::ResultBase;

public:
  constexpr Result() noexcept {}
  constexpr Result(psm::Error e) noexcept : Base(e) {}

  constexpr Result(const Result &other) noexcept : Base(other) {}

  constexpr Result(Result &&other) noexcept : Base(other) {}

  constexpr Result &operator=(const Result &other) noexcept {
    Base::operator=(other);
    return *this;
  }

  constexpr Result &operator=(Result &&other) noexcept {
    Base::operator=(std::move(other));
    return *this;
  }

  /// returns true if the error() == Error::none.
  constexpr operator bool() const noexcept { return Base::operator bool(); }

  /// returns true if the error() == Error::none.
  constexpr bool has_value() const noexcept { return static_cast<bool>(*this); }

  /// get the error associated with the result. If this contains a value,
  /// Error::none is returned.
  constexpr Error error() const noexcept { return Base::error(); }
};
} // namespace psm
#endif /* RESULT_HPP_ */
