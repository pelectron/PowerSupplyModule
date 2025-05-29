#ifndef HAL_CALLBACK_HPP
#define HAL_CALLBACK_HPP

#include "cli/util.hpp"
#include <tuple>
#include <type_traits>
namespace hal {

class CallbackBase {
public:
  constexpr void operator()() {
    if (this->obj)
      this->func(this->obj);
  }

private:
  friend class CallbackList;
  template <size_t Size, size_t Align, class... Args> friend class Callback;

  void *obj = nullptr;
  void (*func)(void *) = nullptr;
  CallbackBase *next = nullptr;
  CallbackBase *prev = nullptr;
};

template <size_t Size, size_t Align, class... Args>
class Callback : public CallbackBase {
public:
  template <typename F> void emplace(F &&f) {
    using T = std::decay_t<F>;
    static_assert(std::is_trivially_destructible_v<T>,
                  "f must be trivially destructible!");
    static_assert(std::is_invocable_v<T, Args...>,
                  "f must be callable with args");

    new ((void *)buffer) T(std::forward<F>(f));
    this->obj = this;
    this->func = &invoke<T>;
  }

  constexpr void reset() {
    this->func = nullptr;
    this->obj = nullptr;
  }

  template <class... A> constexpr void set_args(A &&...args) {
    this->args = std::tuple{std::forward<A>(args)...};
  }

  template <class... A> constexpr void operator()(Args... args) {
    set_args(std::forward<A>(args)...);
    Callback::operator()();
  }

private:
  using This = Callback<Size, Align, Args...>;

  template <class T> static void invoke(void *obj) {
    This *this_ = std::launder(reinterpret_cast<This *>(obj));
    std::apply(*std::launder(reinterpret_cast<T *>(this_->buffer)),
               this_->args);
  }
  std::tuple<Args...> args;
  alignas(Align) std::byte buffer[Size]{};
};

template <size_t Size, size_t Align>
class Callback<Size, Align> : public CallbackBase {
public:
  template <typename F> void emplace(F &&f) {
    using T = std::decay_t<F>;
    static_assert(std::is_trivially_destructible_v<T>,
                  "f must be trivially destructible!");
    static_assert(std::is_invocable_v<T>,
                  "f must be callable without any arguments");
    T *obj = new ((void *)buffer) T(std::forward<F>(f));
    this->obj = obj;
    this->func = &invoke<T>;
  }

  constexpr void reset() {
    this->func = nullptr;
    this->obj = nullptr;
  }

private:
  template <class T> static void invoke(void *obj) {
    (*std::launder(reinterpret_cast<T *>(obj)))();
  }

  alignas(Align) std::byte buffer[Size]{};
};

class CallbackList {
public:
  constexpr bool add_callback(CallbackBase &callback) {
    if (callback.next or callback.prev)
      return false;

    if (tail_ == nullptr) {
      head_ = &callback;
      tail_ = head_;
      return true;
    }
    tail_->next = &callback;
    callback.prev = tail_;
    tail_ = &callback;
    return true;
  }

  constexpr bool remove_callback(CallbackBase &callback) {
    if (callback.next == nullptr and callback.prev == nullptr)
      return false;
    CallbackBase *it = head_;
    while (it) {
      if (it == &callback) {
        auto prev = it->prev;
        auto next = it->next;
        if (prev == nullptr) {
          // it is first
          head_ = next;
        } else {
          prev->next = next;
          it->prev = nullptr;
        }
        if (next == nullptr) {
          // it is last
          tail_ = prev;
        } else {
          next->prev = prev;
          it->next = nullptr;
        }
        return true;
      }
    }
    return false;
  }

  constexpr void invoke() {
    CallbackBase *it = head_;
    while (it) {
      it->func(it->obj);
      it = it->next;
    }
  }

private:
  CallbackBase *head_ = nullptr;
  CallbackBase *tail_ = nullptr;
};

} // namespace hal
#endif
