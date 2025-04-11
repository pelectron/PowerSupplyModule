/*
 * fixed_vector.hpp
 *
 *  Created on: Mar 13, 2024
 *      Author: pelec
 */

#ifndef FIXED_VECTOR_HPP
#define FIXED_VECTOR_HPP
#include "memory.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

namespace psm {

template <typename T, typename = void> struct is_iterator : std::false_type {};
template <typename T>
struct is_iterator<
    T, std::void_t<typename std::iterator_traits<T>::iterator_category>>
    : std::true_type {};
template <typename T>
inline constexpr bool is_iterator_v = is_iterator<T>::value;

namespace fixed_vec_dtl {
struct Empty {
  constexpr Empty() = default;
  constexpr Empty(const Empty &) = default;
  constexpr Empty(Empty &&) = default;
  constexpr Empty &operator=(const Empty &) = default;
  constexpr Empty &operator=(Empty &&) = default;
};

/// the type of elements stored in a non trivial fixed_vec.
template <typename T> union Elem {
  constexpr Elem() : e() {}
  constexpr ~Elem() {}

  Empty e;
  T t;
};

// fixed vec iterator
template <typename T> class iterator {
public:
  template <typename, size_t, bool> friend class uninit_storage;
  using value_type = T;
  using difference_type = std::make_signed_t<size_t>;
  using reference = T &;
  constexpr reference operator*() const { return elem->t; }
  constexpr reference operator[](difference_type i) { return elem[i].t; }

  constexpr iterator &operator+=(difference_type n) {
    elem += n;
    return *this;
  }
  constexpr iterator &operator-=(difference_type n) {
    elem -= n;
    return *this;
  }

  constexpr iterator &operator++() {
    ++elem;
    return *this;
  }
  constexpr iterator operator++(int) {
    iterator ret = *this;
    ++elem;
    return ret;
  }
  constexpr iterator &operator--() {
    --elem;
    return *this;
  }
  constexpr iterator operator--(int) {
    iterator ret = *this;
    --elem;
    return ret;
  }
  friend constexpr iterator operator+(iterator it, difference_type n) {
    return iterator{it.elem + n};
  }
  friend constexpr iterator operator+(difference_type n, iterator it) {
    return iterator{it.elem + n};
  }
  friend constexpr difference_type operator-(iterator b, iterator a) {
    return b.elem - a.elem;
  }
  friend constexpr bool operator<(iterator it1, iterator it2) {
    return it1.elem < it2.elem;
  }
  friend constexpr bool operator<=(iterator it1, iterator it2) {
    return it1.elem <= it2.elem;
  }
  friend constexpr bool operator>(iterator it1, iterator it2) {
    return it1.elem > it2.elem;
  }
  friend constexpr bool operator>=(iterator it1, iterator it2) {
    return it1.elem >= it2.elem;
  }
  friend constexpr bool operator==(iterator it1, iterator it2) {
    return it1.elem == it2.elem;
  }
  template <typename> friend class const_iterator;

private:
  constexpr iterator(fixed_vec_dtl::Elem<T> *elem) : elem(elem) {}
  fixed_vec_dtl::Elem<T> *elem;
};

// fixed vec const iterator
template <typename T> class const_iterator {
public:
  template <typename, size_t, bool> friend class uninit_storage;
  using value_type = T const;
  using difference_type = std::make_signed_t<size_t>;
  using reference = T const &;
  constexpr const_iterator(const iterator<T> &it) noexcept : elem(it.elem) {}

  constexpr reference operator*() const { return elem->t; }
  constexpr reference operator[](size_t i) { return elem[i].t; }

  constexpr const_iterator &operator+=(difference_type n) {
    elem += n;
    return *this;
  }
  constexpr const_iterator &operator-=(difference_type n) {
    elem -= n;
    return *this;
  }

  constexpr const_iterator &operator++() {
    ++elem;
    return *this;
  }
  constexpr const_iterator operator++(int) {
    const_iterator ret = *this;
    ++elem;
    return ret;
  }
  constexpr const_iterator &operator--() {
    --elem;
    return *this;
  }
  constexpr const_iterator operator--(int) {
    const_iterator ret = *this;
    --elem;
    return ret;
  }
  friend constexpr const_iterator operator+(const_iterator it,
                                            difference_type n) {
    return const_iterator{it.elem + n};
  }
  friend constexpr const_iterator operator+(difference_type n,
                                            const_iterator it) {
    return const_iterator{it.elem + n};
  }
  friend constexpr difference_type operator-(const_iterator b,
                                             const_iterator a) {
    return b.elem - a.elem;
  }
  friend constexpr bool operator<(const_iterator it1, const_iterator it2) {
    return it1.elem < it2.elem;
  }
  friend constexpr bool operator<=(const_iterator it1, const_iterator it2) {
    return it1.elem <= it2.elem;
  }
  friend constexpr bool operator>(const_iterator it1, const_iterator it2) {
    return it1.elem > it2.elem;
  }
  friend constexpr bool operator>=(const_iterator it1, const_iterator it2) {
    return it1.elem >= it2.elem;
  }
  friend constexpr bool operator==(const_iterator it1, const_iterator it2) {
    return it1.elem == it2.elem;
  }

private:
  constexpr const_iterator(const fixed_vec_dtl::Elem<T> *elem) : elem(elem) {}
  const fixed_vec_dtl::Elem<T> *elem;
};

/// uninitialized storage for Size elements of type T.
/// It essentially is a c array of type Elem<T>[Size] to ensure no default
/// initialization its elements.
template <typename T, size_t Size, bool trivial> class uninit_storage {
public:
  using iter = fixed_vec_dtl::iterator<T>;
  using const_iter = fixed_vec_dtl::const_iterator<T>;

  constexpr uninit_storage() = default;

  template <typename... Args>
  constexpr void construct_new(size_t pos, Args &&...args) {
    psm::construct_at(&data_[pos].t, std::forward<Args>(args)...);
  }
  constexpr void construct_existing(size_t pos, const T &t) {
    data_[pos].t = t;
  }
  constexpr void construct_existing(size_t pos, T &&t) {
    data_[pos].t = std::move(t);
  }

  constexpr void destroy(size_t pos) {
    std::destroy_at(&((data_ + pos)->t));
    (data_ + pos)->e = Empty{};
  }

  constexpr iter it(size_t pos) noexcept { return data_ + pos; }

  constexpr const_iter it(size_t pos) const noexcept { return data_ + pos; }

private:
  fixed_vec_dtl::Elem<T> data_[Size];
};

template <typename T, size_t Capacity, bool trivial> class fixed_vec {
  using storage = fixed_vec_dtl::uninit_storage<T, Capacity, trivial>;
  template <typename, size_t, bool> friend class fixed_vec;

public:
  // types:
  using value_type = T;
  using pointer = T *;
  using const_pointer = T const *;
  using reference = value_type &;
  using const_reference = const value_type &;
  using size_type = size_t /*5.9 smallest unsigned integer type that can
                              represent C (Capacity)*/
      ;
  using difference_type = std::make_signed_t<size_type>;
  using iterator = fixed_vec_dtl::iterator<T>; // see [container.requirements]
  using const_iterator =
      fixed_vec_dtl::const_iterator<T>; // see [container.requirements]
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  // 5.2, copy/move construction:
  /// constructs an empty vector
  constexpr fixed_vec() noexcept : data_{}, size_{0} {}

  /// constructs a vector with n elements copy constructed from value
  /// @param n number of elements to construct
  /// @param value value to copy construct elements from
  constexpr fixed_vec(size_type n, const value_type &value)
      : data_{}, size_{0} {
    assert(n <= Capacity);
    for (; size_ < n; ++size_) {
      data_.construct_new(size_, value);
    }
  }

  /// constructa vector from the range [first, last).
  template <class InputIterator,
            typename = std::enable_if_t<is_iterator_v<InputIterator>>>
  constexpr fixed_vec(InputIterator first, InputIterator last)
      : data_{}, size_(0) {
    assert((last - first) <= difference_type(Capacity));
    for (; first != last; ++first, ++size_) {
      data_.construct_new(size_, *first);
    }
  }

  constexpr fixed_vec(fixed_vec const &other) noexcept(
      std::is_nothrow_copy_constructible<value_type>{})
      : data_{}, size_(0) {
    for (; size_ < other.size(); ++size_) {
      data_.construct_new(size_, other[size_]);
    }
  }

  template <size_t C>
  constexpr fixed_vec(fixed_vec<T, C, trivial> const &other) noexcept(
      std::is_nothrow_copy_constructible<value_type>{})
      : data_{}, size_(0) {
    for (; size_ < other.size(); ++size_) {
      data_.construct_new(size_, other[size_]);
    }
  }

  constexpr fixed_vec(fixed_vec &&other) noexcept(
      std::is_nothrow_move_constructible<value_type>{})
      : data_{}, size_{0} {

    // move over elements
    for (; size_ < other.size(); ++size_) {
      data_.construct_new(size_, std::move(other[size_]));
    }
    // destroy moved from elements in old vector.
    other.clear();
  }

  template <size_t C>
  constexpr fixed_vec(fixed_vec<T, C, trivial> &&other) noexcept(
      std::is_nothrow_move_constructible<value_type>{})
      : data_{}, size_{0} {
    static_assert(C <= Capacity);
    // move over elements
    for (; size_ < other.size(); ++size_) {
      data_.construct_new(size_, std::move(other[size_]));
    }
    // destroy moved from elements in old vector.
    other.clear();
  }

  constexpr fixed_vec(std::initializer_list<value_type> il) {
    assert(il.size() <= Capacity);
    for (; size_ < il.size(); ++size_) {
      data_.construct_new(size_, *(std::data(il) + size_));
    }
  }

  constexpr ~fixed_vec() { clear(); }

  // 5.3, copy/move assignment:
  constexpr fixed_vec &operator=(fixed_vec const &other) noexcept(
      std::is_nothrow_copy_assignable<value_type>{}) {
    copy(other);
    return *this;
  }

  template <size_t C>
  constexpr fixed_vec &
  operator=(fixed_vec<T, C, trivial> const &other) noexcept(
      std::is_nothrow_copy_assignable<value_type>{}) {
    copy(other);
    return *this;
  }

  constexpr fixed_vec &operator=(fixed_vec &&other) noexcept(
      std::is_nothrow_move_assignable<value_type>{}) {
    // move over elements
    for (; size_ < other.size(); ++size_) {
      data_.construct_new(size_, std::move(other[size_]));
    }
    // destroy moved from elements in old vector.
    other.clear();
    return *this;
  }

  template <size_t C>
  constexpr fixed_vec &operator=(fixed_vec<T, C, trivial> &&other) noexcept(
      std::is_nothrow_move_assignable<value_type>{}) {
    // move over elements
    for (; size_ < other.size(); ++size_) {
      data_.construct_new(size_, std::move(other[size_]));
    }
    // destroy moved from elements in old vector.
    other.clear();
    return *this;
  }

  template <class InputIterator,
            typename = std::enable_if_t<is_iterator_v<InputIterator>>>
  constexpr void assign(InputIterator first, InputIterator last) {
    clear();
    assert((last - first) <= difference_type(Capacity));
    for (; first != last; ++first, ++size_) {
      data_.construct_new(size_, *first);
    }
  }
  constexpr void assign(size_type n, const value_type &u) {
    assert(n <= Capacity);
    clear();
    for (; size_ < n; ++size_) {
      data_.construct_new(size_, u);
    }
  }
  constexpr void assign(std::initializer_list<value_type> il) {
    assert(il.size() <= Capacity);
    clear();
    for (; size_ < il.size(); ++size_) {
      data_.construct_new(size_, *(std::data(il) + size_));
    }
  }

  // iterators
  constexpr iterator begin() noexcept { return data_.it(0); }
  constexpr const_iterator begin() const noexcept { return data_.it(0); }
  constexpr iterator end() noexcept { return data_.it(size_); }
  constexpr const_iterator end() const noexcept { return data_.it(size_); }

  constexpr reverse_iterator rbegin() noexcept { return end(); }
  constexpr const_reverse_iterator rbegin() const noexcept { return end(); }
  constexpr reverse_iterator rend() noexcept { return begin(); }
  constexpr const_reverse_iterator rend() const noexcept { return begin(); }
  constexpr const_iterator cbegin() noexcept { return begin(); }
  constexpr const_iterator cend() const noexcept { return end(); }
  constexpr const_reverse_iterator crbegin() noexcept { return rbegin(); }
  constexpr const_reverse_iterator crend() const noexcept { return rend(); }

  // 5.5, size/capacity:
  constexpr bool empty() const noexcept { return size_ == 0; }
  constexpr size_type size() const noexcept { return size_; }
  static constexpr size_type max_size() noexcept { return Capacity; }
  static constexpr size_type capacity() noexcept { return Capacity; }
  constexpr bool shrink(size_type sz) {
    assert(sz <= Capacity);
    if (sz > size_)
      return false;

    for (; size_ > sz; --size_) {
      data_.destroy(size_ - 1);
    }
    return true;
  }
  constexpr void resize(size_type sz, const value_type &c) {
    if (sz < size_) {
      shrink(sz);
      return;
    }
    for (; size_ <= sz; ++size_) {
      data_.construct_new(size_, c);
    }
  }

  // 5.6, element and data access:
  constexpr reference operator[](size_type n) noexcept {
    assert(n < size_);
    return *data_.it(n);
  }
  constexpr const_reference operator[](size_type n) const noexcept {
    assert(n < size_);
    return *data_.it(n);
  }
  constexpr const_reference at(size_type n) const {
    assert(n < size_);
    return *data_.it(n);
  }
  constexpr reference at(size_type n) {
    assert(n < size_);
    return *data_.it(n);
  }
  constexpr reference front() noexcept {
    assert(size_ > 0);
    return *begin();
  }
  constexpr const_reference front() const noexcept {
    assert(size_ > 0);
    return *begin();
  }
  constexpr reference back() noexcept {
    assert(size_ > 0);
    return *data_.it(size_ - 1);
  }
  constexpr const_reference back() const noexcept {
    assert(size_ > 0);
    return *data_.it(size_ - 1);
  }

  // 5.7, modifiers:
  constexpr bool insert(const_iterator position, const value_type &x) {
    return emplace(position, x);
  }
  constexpr bool insert(const_iterator position, value_type &&x) {
    return emplace(position, std::move(x));
  }
  constexpr bool insert(const_iterator position, size_type n,
                        const value_type &x) {
    if ((size_ + n) > Capacity)
      return false;
    const size_type new_size = size_ + n;
    const size_type idx = position - begin();
    const size_type num_elems_after_pos = size_ - idx - 1;
    if (idx == size_) {
      // position == cend()
      for (size_type i = 0; i < n; ++i) {
        push_back(x);
      }
      return true;
    }
    const size_type construct_new = std::min(n, num_elems_after_pos);
    const size_type construct_existing = std::max(n, num_elems_after_pos);
    // n new elements to create, move existing ones back to front
    for (size_type i = 0; i < construct_new; ++i) {
      data_.construct_new(new_size - i - 1,
                          std::move(*data_.it(size_ - i - 1)));
    }
    // the remaining elements already have a moved from place
    for (size_type i = construct_new; i < construct_existing; ++i) {
      data_.construct_existing(new_size - i - 1,
                               std::move(*data_.it(size_ - i - 1)));
    }
    for (size_type i = idx; i < (idx + n); ++i) {
      data_.construct_existing(i, x);
    }
    size_ = new_size;
    return true;
  }

  template <class... Args>
  constexpr bool emplace(const_iterator position, Args &&...args) {
    if (size_ == Capacity) {
      // no more space left
      return false;
    }

    size_type idx = position - cbegin();

    if (idx == size_) {
      // position == cend()
      return this->emplace_back(std::forward<Args>(args)...);
    }

    // move last element one place back, must construct new
    data_.construct_new(size_, std::move(*data_.it(size_ - 1)));

    // move up rest of elements by std::move
    for (size_t i = size_ - 1; i > idx; --i) {
      data_.construct_existing(i, std::move(*data_.it(i - 1)));
    }
    // construc the new element
    data_.construct_existing(idx, T(std::forward<Args>(args)...));
    ++size_;
    return true;
  }

  template <class... Args> constexpr bool emplace_back(Args &&...args) {
    if (size_ == Capacity)
      return false;
    data_.construct_new(size_, std::forward<Args>(args)...);
    ++size_;
    return true;
  }

  constexpr bool push_back(const value_type &x) {
    if (size_ == Capacity)
      return false;
    data_.construct_new(size_, x);
    ++size_;
    return true;
  }

  constexpr bool push_back(value_type &&x) {
    if (size_ == Capacity)
      return false;
    data_.construct_new(size_, std::move(x));
    ++size_;
    return true;
  }

  constexpr void pop_back() {
    data_.destroy(size_ - 1);
    --size_;
  }

  constexpr void erase(const_iterator position) {
    size_type idx = position - begin();
    for (auto i = idx; i < size_ - 1; ++i) {
      data_.construct_existing(i, std::move(*data_.it(i + 1)));
    }
    data_.destroy(size_ - 1);
    --size_;
  }

  constexpr void erase(const_iterator first, const_iterator last) {
    const size_type begin = first - cbegin();
    const size_type end = last - cbegin();
    if (last == cend()) {
      for (; size_ > begin; --size_) {
        data_.destroy(size_ - 1);
      }
      return;
    }
    for (size_t i = begin; i < end; ++i) {
      data_.construct_existing(i, std::move(*data_.it(end + i)));
    }
    for (size_type i = size_; end < i; --i) {
      data_.destroy(i - 1);
    }
    size_ = last - first;
  }

  constexpr void clear() noexcept(std::is_nothrow_destructible<value_type>{}) {
    for (; size_ > 0; --size_) {
      data_.destroy(size_ - 1);
    }
    size_ = 0;
  }

private:
  template <size_t C>
  constexpr void copy(const fixed_vec<T, C, trivial> &other) {
    static_assert(C <= Capacity);
    if constexpr (C == Capacity) {
      if (this == &other)
        return;
    }
    // 1. clear this
    this->clear();
    // 2. copy over elements
    for (size_ = 0; size_ < other.size(); ++size_) {
      data_.construct_new(size_, other[size_]);
    }
  }

  template <size_t C> constexpr void move(fixed_vec<T, C, trivial> &&other) {
    static_assert(C <= Capacity);
    if constexpr (C == Capacity) {
      if (this == &other)
        return;
    }

    // 1. clear this
    this->clear();

    // 2. move over elements
    for (; size_ < other.size(); ++size_) {
      data_.construct_new(size_, std::move(other[size_]));
    }
    // destroy moved from elements in old vector.
    other.clear();
  }

  storage data_{};
  size_t size_{0};
};

template <typename T, size_t Capacity> class fixed_vec<T, Capacity, true> {
  using storage = fixed_vec_dtl::uninit_storage<T, Capacity, true>;
  template <typename, size_t, bool> friend class fixed_vec;

public:
  // types:
  using value_type = T;
  using pointer = T *;
  using const_pointer = T const *;
  using reference = value_type &;
  using const_reference = const value_type &;
  using size_type = size_t /*5.9 smallest unsigned integer type that can
                              represent C (Capacity)*/
      ;
  using difference_type = std::make_signed_t<size_type>;
  using iterator = T *;             // see [container.requirements]
  using const_iterator = T const *; // see [container.requirements]
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  // 5.2, copy/move construction:
  /// constructs an empty vector
  constexpr fixed_vec() noexcept : data_{}, size_{0} {}

  /// constructs a vector with n default initialized elements
  /// @param n number of elements to construct
  constexpr explicit fixed_vec(size_type n) : data_{}, size_{n} {}

  /// constructs a vector with n elements copy constructed from value
  /// @param n number of elements to construct
  /// @param value value to copy construct elements from
  constexpr fixed_vec(size_type n, const value_type &value)
      : data_{}, size_{0} {
    assert(n <= Capacity);
    for (; size_ < n; ++size_) {
      data_[size_] = value;
    }
  }

  /// constructa vector from the range [first, last).
  template <class InputIterator,
            typename = std::enable_if_t<is_iterator_v<InputIterator>>>
  constexpr fixed_vec(InputIterator first, InputIterator last)
      : data_{}, size_(0) {
    assert((last - first) <= difference_type(Capacity));
    for (; first != last; ++first, ++size_) {
      data_[size_] = *first;
    }
  }

  constexpr fixed_vec(fixed_vec const &other) noexcept(
      std::is_nothrow_copy_constructible<value_type>{})
      : data_{}, size_(0) {
    copy(other);
  }

  // template <size_t C>
  // constexpr fixed_vec(fixed_vec<T, C, true> const &other) noexcept(
  //     std::is_nothrow_copy_constructible<value_type>{})
  //     : data_{}, size_(0) {
  //   copy(other);
  // }

  constexpr fixed_vec(fixed_vec &&other) noexcept(
      std::is_nothrow_move_constructible<value_type>{})
      : data_{}, size_{0} {
    move(std::move(other));
  }

  // template <size_t C>
  // constexpr fixed_vec(fixed_vec<T, C, true> &&other) noexcept(
  //     std::is_nothrow_move_constructible<value_type>{})
  //     : data_{}, size_{0} {
  //   move(std::move(other));
  // }

  constexpr fixed_vec(std::initializer_list<value_type> il) {
    assert(il.size() <= Capacity);
    for (const auto &v : il) {
      data_[size_] = v;
      ++size_;
    }
  }

  // 5.3, copy/move assignment:
  constexpr fixed_vec &operator=(fixed_vec const &other) noexcept(
      std::is_nothrow_copy_assignable<value_type>{}) {
    copy(other);
    return *this;
  }

  template <size_t C>
  constexpr fixed_vec &operator=(fixed_vec<T, C, true> const &other) noexcept(
      std::is_nothrow_copy_assignable<value_type>{}) {
    copy(other);
    return *this;
  }

  constexpr fixed_vec &operator=(fixed_vec &&other) noexcept(
      std::is_nothrow_move_assignable<value_type>{}) {
    move(std::move(other));
    return *this;
  }

  template <size_t C>
  constexpr fixed_vec &operator=(fixed_vec<T, C, true> &&other) noexcept(
      std::is_nothrow_move_assignable<value_type>{}) {
    move(std::move(other));
    return *this;
  }

  template <class InputIterator,
            typename = std::enable_if_t<is_iterator_v<InputIterator>>>
  constexpr void assign(InputIterator first, InputIterator last) {
    size_ = 0;
    for (; first != last; ++first, ++size_) {
      assert(size_ <= Capacity);
      data_[size_] = *first;
    }
  }

  constexpr void assign(size_type n, const value_type &u) {
    assert(n <= Capacity);
    size_ = 0;
    for (size_ = 0; size_ < n; ++size_) {
      data_[size_] = u;
    }
  }

  constexpr void assign(std::initializer_list<value_type> il) {
    assert(il.size() <= Capacity);
    size_ = 0;
    for (const auto &v : il) {
      data_[size_] = v;
      ++size_;
    }
  }

  // iterators
  constexpr iterator begin() noexcept { return data_; }
  constexpr const_iterator begin() const noexcept { return data_; }
  constexpr iterator end() noexcept { return data_ + size_; }
  constexpr const_iterator end() const noexcept { return data_ + size_; }

  constexpr reverse_iterator rbegin() noexcept { return end(); }
  constexpr const_reverse_iterator rbegin() const noexcept { return end(); }
  constexpr reverse_iterator rend() noexcept { return begin(); }
  constexpr const_reverse_iterator rend() const noexcept { return begin(); }
  constexpr const_iterator cbegin() noexcept { return begin(); }
  constexpr const_iterator cend() const noexcept { return end(); }
  constexpr const_reverse_iterator crbegin() noexcept { return rbegin(); }
  constexpr const_reverse_iterator crend() const noexcept { return rend(); }

  // 5.5, size/capacity:
  constexpr bool empty() const noexcept { return size_ == 0; }
  constexpr size_type size() const noexcept { return size_; }
  static constexpr size_type max_size() noexcept { return Capacity; }
  static constexpr size_type capacity() noexcept { return Capacity; }
  constexpr bool shrink(size_type sz) {
    assert(sz <= Capacity);
    if (sz > size_)
      return false;

    size_ = sz;
    return true;
  }

  constexpr void resize(size_type sz, const value_type &c) {
    if (sz < size_) {
      size_ = sz;
      return;
    }
    for (; size_ < sz; ++size_) {
      data_[size_] = c;
    }
  }

  // 5.6, element and data access:
  constexpr reference operator[](size_type n) noexcept {
    assert(n < size_);
    return data_[n];
  }
  constexpr const_reference operator[](size_type n) const noexcept {
    assert(n < size_);
    return data_[n];
  }
  constexpr const_reference at(size_type n) const {
    assert(n < size_);
    return data_[n];
  }
  constexpr reference at(size_type n) {
    assert(n < size_);
    return data_[n];
  }
  constexpr reference front() noexcept {
    assert(size_ > 0);
    return data_[0];
  }
  constexpr const_reference front() const noexcept {
    assert(size_ > 0);
    return data_[0];
  }
  constexpr reference back() noexcept {
    assert(size_ > 0);
    return data_[size_ - 1];
  }
  constexpr const_reference back() const noexcept {
    assert(size_ > 0);
    return data_[size_ - 1];
  }
  // 5.7, modifiers:
  constexpr bool insert(const_iterator position, const value_type &x) {
    return emplace(position, x);
  }
  constexpr bool insert(const_iterator position, value_type &&x) {
    return emplace(position, std::move(x));
  }
  constexpr bool insert(const_iterator position, size_type n,
                        const value_type &x) {
    const size_type new_size = size_ + n;
    if (new_size > Capacity)
      return false;
    const size_type idx = position - data_;
    const size_type num_elems_after_pos = size_ - idx;
    // constructs new elements by moving existing ones back to front (n new
    // elements added, but can only construct num_elems_after_pos elements
    // max)
    for (size_type i = 0; i < num_elems_after_pos; ++i) {
      data_[new_size - i - 1] = std::move(data_[size_ - i - 1]);
    }
    for (size_type i = 0; i < n; ++i) {
      data_[i + idx] = x;
    }
    size_ = new_size;
    return true;
  }

  template <class... Args>
  constexpr bool emplace(const_iterator position, Args &&...args) {
    if (size_ == Capacity) {
      // no more space left
      return false;
    }

    size_type idx = position - data_;

    // move up elements after position by std::move back to front
    for (size_t i = size_; i > idx; --i) {
      data_[i] = std::move(data_[i - 1]);
    }
    // construct the new element
    data_[idx] = T(std::forward<Args>(args)...);
    ++size_;
    return true;
  }

  template <class... Args> constexpr bool emplace_back(Args &&...args) {
    if (size_ == Capacity)
      return false;
    data_[size_] = T(std::forward<Args>(args)...);
    ++size_;
    return true;
  }

  constexpr bool push_back(const value_type &x) {
    if (size_ == Capacity)
      return false;
    data_[size_] = x;
    ++size_;
    return true;
  }

  constexpr bool push_back(value_type &&x) {
    if (size_ == Capacity)
      return false;
    data_[size_] = std::move(x);
    ++size_;
    return true;
  }

  constexpr void pop_back() { --size_; }

  constexpr void erase(const_iterator position) {
    if (size_ == 0)
      return;
    size_type idx = position - begin();
    for (size_type i = idx; i < size_ - 1; ++i) {
      data_[i] = std::move(data_[i + 1]);
    }
    --size_;
  }

  constexpr void erase(const_iterator first, const_iterator last) {
    const size_type n = last - first;
    if (size_ == 0 or size_ < n)
      return;
    const size_type start = first - data_;
    const size_type idx = last - begin();
    for (size_type i = idx; i < size_; ++i) {
      data_[start + i] = data_[i];
    }
    size_ -= n;
  }

  constexpr void clear() noexcept(std::is_nothrow_destructible<value_type>{}) {
    size_ = 0;
  }

private:
  template <size_t C> constexpr void copy(const fixed_vec<T, C, true> &other) {
    static_assert(C <= Capacity);
    if constexpr (C == Capacity) {
      if (this == &other)
        return;
    }
    for (size_ = 0; size_ < other.size_; ++size_) {
      data_[size_] = other.data_[size_];
    }
  }

  template <size_t C> constexpr void move(fixed_vec<T, C, true> &&other) {
    static_assert(C <= Capacity);
    if constexpr (C == Capacity) {
      if (this == &other)
        return;
    }
    for (size_ = 0; size_ < other.size_; ++size_) {
      data_[size_] = other.data_[size_];
    }
    other.size_ = 0;
  }

  T data_[Capacity]{};
  size_t size_{0};
};

template <typename T, size_t Capacity>
using fixed_vec_impl = fixed_vec_dtl::fixed_vec<T, Capacity, is_trivial_v<T>>;
} // namespace fixed_vec_dtl

/**
 * This class is a dynamic array, like std::vector, but with a fixed maximum
 * size. The elements are stored in line, and no extra free store allocations
 * occur when using this class. When compiling with c++17,
 */
template <typename T, size_t Capacity>
class FixedVec : private fixed_vec_dtl::fixed_vec_impl<T, Capacity> {
  template <typename, size_t> friend class fixed_vec;
  using Base = fixed_vec_dtl::fixed_vec_impl<T, Capacity>;

public:
  // types:
  using value_type = T;
  using pointer = T *;
  using const_pointer = T const *;
  using reference = value_type &;
  using const_reference = const value_type &;
  using size_type = size_t /*5.9 smallest unsigned integer type that can
                              represent C (Capacity)*/
      ;
  using difference_type = std::make_signed_t<size_type>;
  using iterator = typename Base::iterator; // see [container.requirements]
  using const_iterator =
      typename Base::const_iterator; // see [container.requirements]
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  // 5.2, copy/move construction:
  /// constructs an empty vector
  constexpr FixedVec() noexcept = default;

  /// constructs a vector with n default initialized elements
  /// @param n number of elements to construct
  template <typename T_ = T, typename = std::enable_if_t<is_trivial_v<T_>>>
  constexpr explicit FixedVec(size_type n) : Base(n) {}

  /// constructs a vector with n elements copy constructed from value
  /// @param n number of elements to construct
  /// @param value value to copy construct elements from
  constexpr FixedVec(size_type n, const value_type &value) : Base(n, value) {}

  /// construct a vector from the range [first, last).
  template <class InputIterator,
            typename = std::enable_if_t<is_iterator_v<InputIterator>>>
  constexpr FixedVec(InputIterator first, InputIterator last)
      : Base(first, last) {}

  template <typename T_ = T,
            typename = std::enable_if_t<std::is_copy_constructible_v<T_>>>
  constexpr FixedVec(FixedVec const &other) noexcept(
      std::is_nothrow_copy_constructible_v<value_type>)
      : Base(other) {}

  template <size_t C, typename T_ = T,
            typename = std::enable_if_t<std::is_copy_constructible_v<T_>>>
  constexpr FixedVec(FixedVec<T, C> const &other) noexcept(
      std::is_nothrow_copy_constructible_v<value_type>)
      : Base(static_cast<const typename FixedVec<T, C>::Base &>(other)) {}

  constexpr FixedVec(FixedVec &&other) noexcept(
      std::is_nothrow_move_constructible_v<value_type>) = default;

  template <size_t C, typename T_ = T,
            typename = std::enable_if_t<std::is_copy_constructible_v<T_>>>
  constexpr FixedVec(FixedVec<T, C> &&other) noexcept(
      std::is_nothrow_move_constructible_v<value_type>)
      : Base(std::move(static_cast<typename FixedVec<T, C>::Base &&>(other))) {}

  constexpr FixedVec(std::initializer_list<value_type> il) noexcept(
      std::is_nothrow_copy_constructible_v<value_type>)
      : Base(il) {}

  /*constexpr ~fixed_vec() */ // implicit. only constexpr if T is trivial or
                              // compiling with c++20 or above.
  // 5.3, copy/move assignment:
  constexpr FixedVec &operator=(FixedVec const &other) noexcept(
      std::is_nothrow_copy_assignable_v<value_type>) {
    Base::operator=(static_cast<const Base &>(other));
    return *this;
  }

  template <size_t C>
  constexpr FixedVec &operator=(FixedVec<T, C> const &other) noexcept(
      std::is_nothrow_copy_assignable_v<value_type>) {
    Base::operator=(static_cast<const typename FixedVec<T, C>::Base &>(other));
    return *this;
  }

  constexpr FixedVec &operator=(FixedVec &&other) noexcept(
      std::is_nothrow_move_assignable_v<value_type>) {
    Base::operator=(std::move(static_cast<Base &&>(other)));
    return *this;
  }

  template <size_t C>
  constexpr FixedVec &operator=(FixedVec<T, C> &&other) noexcept(
      std::is_nothrow_move_assignable<value_type>{}) {
    Base::operator=(
        std::move(static_cast<typename FixedVec<T, C>::Base &&>(other)));
    return *this;
  }

  template <class InputIterator,
            typename = std::enable_if_t<is_iterator_v<InputIterator>>>
  constexpr void assign(InputIterator first, InputIterator last) {
    Base::assign(first, last);
  }

  constexpr void assign(size_type n, const value_type &u) {
    Base::assign(n, u);
  }

  constexpr void assign(std::initializer_list<value_type> il) {
    Base::assign(il);
  }

  // iterators
  constexpr iterator begin() noexcept { return Base::begin(); }
  constexpr const_iterator begin() const noexcept { return Base::begin(); }
  constexpr iterator end() noexcept { return Base::end(); }
  constexpr const_iterator end() const noexcept { return Base::end(); }
  constexpr reverse_iterator rbegin() noexcept { return Base::rbegin(); }
  constexpr const_reverse_iterator rbegin() const noexcept {
    return Base::rbegin();
  }
  constexpr reverse_iterator rend() noexcept { return Base::rend(); }
  constexpr const_reverse_iterator rend() const noexcept {
    return Base::rend();
  }
  constexpr const_iterator cbegin() noexcept { return Base::cbegin(); }
  constexpr const_iterator cend() const noexcept { return Base::cend(); }
  constexpr const_reverse_iterator crbegin() noexcept { return Base::rbegin(); }
  constexpr const_reverse_iterator crend() const noexcept {
    return Base::rend();
  }

  // 5.5, size/capacity:
  constexpr bool empty() const noexcept { return Base::empty(); }
  constexpr size_type size() const noexcept { return Base::size(); }
  static constexpr size_type max_size() noexcept { return Capacity; }
  static constexpr size_type capacity() noexcept { return Capacity; }
  constexpr bool shrink(size_type sz) { return Base::shrink(sz); }
  constexpr void resize(size_type sz, const value_type &c) {
    Base::resize(sz, c);
  }

  // 5.6, element and data access:
  constexpr reference operator[](size_type n) noexcept {
    return Base::operator[](n);
  }
  constexpr const_reference operator[](size_type n) const noexcept {
    return Base::operator[](n);
  }
  constexpr const_reference at(size_type n) const { return Base::at(n); }
  constexpr reference at(size_type n) { return Base::at(n); }
  constexpr reference front() noexcept { return Base::front(); }
  constexpr const_reference front() const noexcept { return Base::front(); }
  constexpr reference back() noexcept { return Base::back(); }
  constexpr const_reference back() const noexcept { return Base::back(); }

  // 5.7, modifiers:
  constexpr bool insert(const_iterator position, const value_type &x) {
    return Base::insert(position, x);
  }
  constexpr bool insert(const_iterator position, value_type &&x) {
    return Base::insert(position, std::move(x));
  }
  constexpr bool insert(const_iterator position, size_type n,
                        const value_type &x) {
    return Base::insert(position, n, x);
  }

  template <class... Args>
  constexpr bool emplace(const_iterator position, Args &&...args) {
    return Base::emplace(position, std::forward<Args>(args)...);
  }

  template <class... Args> constexpr bool emplace_back(Args &&...args) {
    return Base::emplace_back(std::forward<Args>(args)...);
  }

  constexpr bool push_back(const value_type &x) { return Base::push_back(x); }

  constexpr bool push_back(value_type &&x) {
    return Base::push_back(std::move(x));
  }

  constexpr void pop_back() { Base::pop_back(); }

  constexpr void erase(const_iterator position) { Base::erase(position); }

  constexpr void erase(const_iterator first, const_iterator last) {
    Base::erase(first, last);
  }

  constexpr void clear() noexcept(std::is_nothrow_destructible<value_type>{}) {
    Base::clear();
  }
};

template <typename T>
using vec_iterator =
    std::conditional_t<is_trivial_v<T>, T *, fixed_vec_dtl::iterator<T>>;

template <typename T>
using vec_const_iterator = std::conditional_t<is_trivial_v<T>, T const *,
                                              fixed_vec_dtl::const_iterator<T>>;
} // namespace psm

#endif /* FIXED_VECTOR_HPP_ */
