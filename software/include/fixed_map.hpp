/*
 * fixed_map.hpp
 *
 *  Created on: Mar 9, 2024
 *      Author: pelec
 */

#ifndef MAP_HPP
#define MAP_HPP

#include "fixed_vector.hpp"
#include "result.hpp"

#include <cassert>
#include <cstddef>
#include <type_traits>

namespace psm {
template <typename Key, typename Value, size_t Size, typename SearchPolicy>
class FixedMap;

template <typename Key, typename Value> struct map_element {
  Key key;
  Value value;
};

template <typename K, typename V>
map_element(K &&, V &&) -> map_element<std::decay_t<K>, std::decay_t<V>>;

template <typename Key, typename Value> class MapView {
public:
  using key_type = Key;
  using value_type = Value;
  using element_type = map_element<Key, Value>;
  constexpr MapView() noexcept;
  constexpr MapView(map_element<Key, Value> *data, size_t size)
      : data_(data), size_(size) {}
  constexpr const element_type *find(const Key &key) const noexcept {
    for (const auto &elem : data_) {
      if (elem.key == key)
        return &elem;
    }
    return nullptr;
  }

  constexpr element_type *find(const Key &key) noexcept {
    for (const auto &elem : data_) {
      if (elem.key == key)
        return &elem;
    }
    return nullptr;
  }

  constexpr bool contains(const Key &key) const noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (data_[i].key == key)
        return true;
    }
    return false;
  }

  constexpr const value_type &operator[](const Key &k) const {
    return this->get(k);
  }

  constexpr value_type &operator[](const Key &k) { return this->get(k); }

  constexpr const value_type &get(const Key &k) const {
    for (size_t i = 0; i < size_; ++i) {
      if (data_[i].key == k)
        return data_[i].value;
    }
    assert(false);
  }

  constexpr value_type &get(const Key &k) {
    for (size_t i = 0; i < size_; ++i) {
      if (data_[i].key == k)
        return data_[i].value;
    }
    assert(false);
  }

  constexpr value_type &get_or(const Key &k, value_type &v) noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (data_[i].key == k)
        return data_[i].value;
    }
    return v;
  }

  constexpr const value_type &get_or(const Key &k,
                                     const value_type &v) const noexcept {
    for (size_t i = 0; i < size_; ++i) {
      if (data_[i].key == k)
        return data_[i].value;
    }
    return v;
  }

  constexpr size_t size() const noexcept { return size_; }

  constexpr element_type *data() noexcept { return data_; }
  constexpr const element_type *data() const noexcept { return data_; }

  constexpr element_type *begin() noexcept { return data_; }
  constexpr const element_type *begin() const noexcept { return data_; }
  constexpr const element_type *cbegin() const noexcept { return data_; }
  constexpr element_type *end() noexcept { return data_ + size_; }
  constexpr const element_type *end() const noexcept { return data_ + size_; }
  constexpr const element_type *cend() const noexcept { return data_ + size_; }

protected:
  void set_data(element_type *e) noexcept { data_ = e; }
  void set_size(size_t s) noexcept { size_ = s; }

private:
  element_type *data_{nullptr};
  size_t size_{0};
};

template <typename Key, typename Value>
MapView(map_element<Key, Value> *data, size_t size) -> MapView<Key, Value>;

namespace map_dtl {

template <typename Key, typename Value> class iterator {
public:
  using value_type = map_element<const Key &, Value &>;
  using difference_type = std::make_signed_t<size_t>;
  using reference = value_type;

  constexpr const Key &key() const { return (*elem).key; }
  constexpr Value &value() { return (*elem).value; }
  constexpr const Value &value() const { return (*elem).value; }
  constexpr reference operator*() const { return {*(elem).key, (*elem).value}; }
  constexpr reference operator[](size_t i) {
    return {elem[i].key, elem[i].value};
  }

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
  template <typename, typename> friend class const_iterator;

private:
  using It = vec_iterator<map_element<Key, Value>>;
  template <typename, typename, size_t, typename> friend class psm::FixedMap;
  iterator(It it) : elem(it) {}
  It elem;
};

template <typename Key, typename Value> class const_iterator {
public:
  using value_type = map_element<const Key &, const Value &>;
  using difference_type = std::make_signed_t<size_t>;
  using reference = value_type;
  constexpr const_iterator(const iterator<Key, Value> &it) : elem(it.elem) {}

  constexpr const Key &key() const { return (*elem).key; }
  constexpr const Value &value() const { return (*elem).value; }

  constexpr reference operator*() const { return {(*elem).key, (*elem).value}; }
  constexpr reference operator[](size_t i) {
    return {elem[i].key, elem[i].value};
  }

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
    iterator ret = *this;
    ++elem;
    return ret;
  }
  constexpr const_iterator &operator--() {
    --elem;
    return *this;
  }
  constexpr const_iterator operator--(int) {
    iterator ret = *this;
    --elem;
    return ret;
  }
  friend constexpr const_iterator operator+(const_iterator it,
                                            difference_type n) {
    return iterator{it.elem + n};
  }
  friend constexpr const_iterator operator+(difference_type n,
                                            const_iterator it) {
    return iterator{it.elem + n};
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
  template <typename, typename, size_t, typename> friend class psm::FixedMap;
  using It = vec_const_iterator<map_element<Key, Value>>;
  const_iterator(It it) : elem(it) {}
  It elem;
};

template <typename Key, typename Value, std::size_t N>
constexpr auto bubble_sort(FixedVec<map_element<Key, Value>, N> v)
    -> FixedVec<map_element<Key, Value>, N> {
  auto n = v.size();
  do {
    size_t new_n = 0;
    for (size_t i = 1; i < n; ++i) {
      if (v[i].key < v[i - 1].key) {
        auto tmp = std::move(v[i - 1]);
        v[i - 1] = std::move(v[i]);
        v[i] = std::move(tmp);
        new_n = i;
      }
    }
    n = new_n;
  } while (n > 0);
  return v;
}

template <typename Key, typename K, typename = void>
struct comparable : std::false_type {};
template <typename Key, typename K>
struct comparable<
    Key, K,
    std::void_t<decltype(std::declval<Key>() < std::declval<K>()),
                decltype(std::declval<K>() < std::declval<Key>()),
                decltype(std::declval<Key>() == std::declval<K>()),
                decltype(std::declval<K>() == std::declval<Key>())>>
    : std::true_type {};

} // namespace map_dtl

struct linear_search {
  template <typename K, typename It>
  constexpr It operator()(const K &key, It begin, It end) const {
    while (begin != end) {
      if ((*begin).key == key)
        return begin;
      ++begin;
    }
    return end;
  }
};

struct binary_search {
  template <typename K, typename It>
  constexpr It operator()(const K &key, It begin, It end) const {
    auto first = begin;
    auto last = end;
    while (first < last) {
      auto middle = first + ((last - first) / 2);
      if ((*middle).key < key) {
        first = ++middle;
      } else if (key < (*middle).key) {
        last = middle;
      } else {
        // key == middle->key
        return middle;
      }
    }
    return end;
  }
};
/**
 * A "dumb" flat map, built on fixed vector.
 * Only useful when a small number of key value pairs needs to be stored
 */
template <typename Key, typename Value, size_t Size,
          typename SearchPolicy = linear_search>
class FixedMap {
public:
  using key_type = Key;
  using mapped_type = Value;
  using element_type = map_element<Key, Value>;
  using iterator = map_dtl::iterator<Key, Value>;
  using const_iterator = map_dtl::const_iterator<Key, Value>;

  using value_type = map_element<Key, Value>;
  using pointer = map_element<Key, Value> *;
  using const_pointer = map_element<Key, Value> const *;
  using reference = typename iterator::reference;
  using const_reference = typename const_iterator::reference;
  using size_type = size_t /*5.9 smallest unsigned integer type that can
                              represent C (Capacity)*/
      ;
  using difference_type = std::make_signed_t<size_type>;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  /// std::true_type if K is less than and equality comparable to Key, else
  /// std::false_type.
  template <typename K> using keyish = map_dtl::comparable<Key, K, void>;
  /// enable_if_t based on keyish<K>::value
  template <typename K>
  using enable_if_keyish = std::enable_if_t<keyish<K>::value>;

  /// std::true_type if V is assignable to Value, else std::false_type.
  template <typename V> using valueish = std::is_assignable<Value &, V>;
  /// enable_if_t based on valueish<V>::value
  template <typename V>
  using enable_if_valueish = std::enable_if_t<valueish<V>::value>;

  /// enable_if_t conjunction of keyish<K> and valueish<V>
  template <typename K, typename V>
  using enable_if_kvish =
      std::enable_if_t<keyish<K>::value and valueish<V>::value>;

  /// constructs an empty map.
  constexpr FixedMap() noexcept = default;

  /**
   * create a map with the key value pairs specified in il. The keys must be
   * unique. This is only asserted, i.e. this is checked only in debug mode.
   * Use psm::fixed_map() free function to also check when not in debug mode.
   * @param il initializer list containing the key value pairs.
   */
  template <typename E = element_type,
            typename = std::enable_if_t<std::is_copy_constructible_v<E>>>
  constexpr FixedMap(std::initializer_list<element_type> il) noexcept(
      std::is_nothrow_copy_constructible_v<map_element<Key, Value>>)
      : data_(map_dtl::bubble_sort(FixedVec<element_type, Size>(il))) {
    for (auto it = data_.cbegin() + 1; it != data_.cend(); ++it) {
      assert((*it).key != (*(it + (-1))).key);
    }
  }
  /**
   * create a map with the key value pairs specified by elems. The keys must be
   * unique. This is only asserted, i.e. this is checked only in debug mode.
   * Use psm::fixed_map() free function to also check when not in debug mode.
   * @param elems vector of key value pairs.
   * @{
   */
  template <typename E = element_type,
            typename = std::enable_if_t<std::is_copy_constructible_v<E>>>
  constexpr FixedMap(
      const FixedVec<map_element<Key, Value>, Size>
          &elems) noexcept(std::
                               is_nothrow_copy_constructible_v<
                                   FixedVec<map_element<Key, Value>, Size>>)
      : data_(map_dtl::bubble_sort(elems)) {
    for (auto it = data_.cbegin() + 1; it != data_.cend(); ++it) {
      assert((*it).key != (*(it + (-1))).key);
    }
  }

  constexpr FixedMap(FixedVec<map_element<Key, Value>, Size> &&elems) noexcept(
      std::is_nothrow_move_constructible_v<
          FixedVec<map_element<Key, Value>, Size>>)
      : data_(map_dtl::bubble_sort(std::move(elems))) {
    for (auto it = data_.cbegin() + 1; it != data_.cend(); ++it) {
      assert((*it).key != *((it + (-1))).key);
    }
  }
  /// @}

  template <typename K, typename V, typename... Ks, typename... Vs>
  constexpr FixedMap(map_element<K, V> e1, map_element<Ks, Vs>... rest) {
    static_assert(sizeof...(Ks) < Size);
    insert_or_replace(std::move(e1.key), std::move(e1.value));
    (insert_or_replace(std::move(rest.key), std::move(rest.value)), ...);
  }
  /// returns true if the map is empty, else false.
  constexpr bool empty() const { return data_.empty(); }

  /// clear the map, i.e. remove all elements. After this call, the map is
  /// empty.
  constexpr void clear() { data_.clear(); }

  /**
   * get iterator to element with the specified key.
   * reutrns end() if key is not found.
   * @tparam K A type comparable to Key
   * @param key the key of the key value pair to find
   * @{
   */
  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr const_iterator find(const K &key) const {
    const SearchPolicy search;
    return search(key, data_.begin(), data_.end());
  }

  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr iterator find(const K &key) {
    const SearchPolicy search;
    return search(key, data_.begin(), data_.end());
  }
  /// @}

  /**
   * insert a new key value pair.
   * If the key is already in the map or the map is full, false is returned and
   * the map is not modified. Else a new element with the key key and the value
   * value is added to the map. returns true if the insertion succeeded, else
   * false.
   *
   * @tparam K type of the key
   * @tparam V type of the value
   * @param key the key to add
   * @param value the value associated with the key
   */
  template <typename K, typename V> constexpr bool insert(K &&key, V &&value) {
    for (auto it = data_.begin(); it != data_.end(); ++it) {
      if ((*it).key == key) {
        // key already exists -> insert failed
        return false;
      } else if (key < (*it).key and
                 (it == data_.begin() ? true : ((*(it + (-1))).key < key))) {
        // if key < (*it).key and key > (--it).key -> found insertion point.
        //--it.key is only accessed when it != data_.begin()
        return data_.emplace(it, std::forward<K>(key), std::forward<V>(value));
      }
    }
    return data_.emplace_back(std::forward<K>(key), std::forward<V>(value));
  }

  /**
   * Either insert the key value pair into the map if the key does not exist, or
   * replace the value associated with key if the key is found. If the map is
   * full, the map is not modified. Returns true if the insertion or replacement
   * succeeded, else false.
   *
   * @tparam K type of the key
   * @tparam V type of the value
   * @param key the key to add or replace the value of
   * @param value the new value associated
   */
  template <typename K, typename V, typename = enable_if_kvish<K &&, V &&>>
  constexpr bool insert_or_replace(K &&key, V &&value) {
    for (auto it = data_.begin(); it != data_.end(); ++it) {
      if ((*it).key == key) {
        (*it).value = std::forward<V>(value);
        return true;
      } else if ((key < (*it).key) and
                 (it == data_.begin() ? true : (*(it + (-1))).key < key)) {
        // if key < (*it).key and key > (--it).key -> found insertion point.
        //--it.key is only accessed when it != data_.begin()
        return data_.emplace(it, std::forward<K>(key), std::forward<V>(value));
      }
    }
    return data_.emplace_back(std::forward<K>(key), std::forward<V>(value));
  }

  /**
   * replace the value associated with key with value. The key must already be
   * in the map, else the replacement fails. Returns true if the value of key
   * could be replaced, else false.
   *
   * @tparam K type of the key
   * @tparam V type of the value
   * @param key the key to replace the value of
   * @param value the new value
   */
  template <typename K, typename V, typename = enable_if_kvish<K &&, V &&>>
  constexpr bool replace(K &&key, V &&value) {
    const SearchPolicy search;
    if (auto it = search(key, data_.begin(), data_.end()); it != data_.end()) {
      (*it).value = std::forward<V>(value);
      return true;
    }
    return false;
  }

  /**
   * remove the key value pair with the key key.
   * Returns true if the key value pair could be removed.
   * Returns false if the key is not found.
   *
   * @tparam K type of the key
   * @param key the key to remove
   */
  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr bool erase(const K &key) {
    const SearchPolicy search;
    if (auto it = search(key, data_.begin(), data_.end()); it != data_.end()) {
      data_.erase(it);
      return true;
    }
    return false;
  }

  /**
   * Access the value associated with key. Undefined behaviour if the key cant
   * be found in the map. The keys presence is asserted in debug mode.
   * Returns a (const) reference to the value associated with key.
   *
   * @tparam K type of the key
   * @param key the key of the value
   * @{
   */
  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr const mapped_type &operator[](const K &key) const {
    return this->cget(key);
  }

  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr mapped_type &operator[](const K &k) {
    return this->get(k);
  }

  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr const mapped_type &get(const K &key) const {
    assert(find(key) != end());
    return find(key).value();
  }

  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr const mapped_type &cget(const K &key) const {
    assert(find(key) != end());
    return find(key).value();
  }

  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr mapped_type &get(const K &key) {
    assert(find(key) != end());
    return find(key).value();
  }
  /// @}

  /**
   * Access the value associated with key, or v if the key is not found.
   * Returns a (const) reference to the value associated with key, or v.
   *
   * @tparam K type of the key
   * @param key the key of the value
   * @param v fallback value
   * @{
   */
  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr mapped_type &get_or(const K &key, mapped_type &v) noexcept {
    if (auto it = find(key); it != end())
      return it.value();
    return v;
  }

  template <typename K, typename = enable_if_keyish<K &&>>
  constexpr const mapped_type &get_or(const K &key,
                                      const mapped_type &v) const noexcept {
    if (auto it = find(key); it != end())
      return it.value();
    return v;
  }
  /// @}

  /// iterators
  /// @{
  constexpr iterator begin() noexcept { return data_.begin(); }
  constexpr const_iterator begin() const noexcept { return data_.begin(); }
  constexpr iterator end() noexcept { return data_.end(); }
  constexpr const_iterator end() const noexcept { return data_.end(); }

  constexpr reverse_iterator rbegin() noexcept { return end(); }
  constexpr const_reverse_iterator rbegin() const noexcept { return end(); }
  constexpr reverse_iterator rend() noexcept { return begin(); }
  constexpr const_reverse_iterator rend() const noexcept { return begin(); }
  constexpr const_iterator cbegin() noexcept { return begin(); }
  constexpr const_iterator cend() const noexcept { return end(); }
  constexpr const_reverse_iterator crbegin() noexcept { return rbegin(); }
  constexpr const_reverse_iterator crend() const noexcept { return rend(); }
  /// @}

  /// returns number of key value pairs stored in the map.
  constexpr size_type size() const noexcept { return data_.size(); }

private:
  template <typename, typename, size_t> friend struct FixedMapObserver;

  FixedVec<element_type, Size> data_;
};

/**
 * create a fixed map from an array of map elements. If pairs contains
 * duplicate keys, an erroneous result is returned. The Key, Value and Size are
 * all deduced.
 * @param pairs the key value pairs used to initialize the map
 * @{
 */
template <size_t Size, typename Key, typename Value>
constexpr Result<FixedMap<Key, Value, Size>>
fixed_map(const map_element<Key, Value> (&pairs)[Size]) {
  for (size_t i = 1; i < Size; ++i) {
    if (pairs[i].key == pairs[i - 1].key)
      return psm::Error::duplicate_key;
  }
  return FixedMap<Key, Value, Size>{std::initializer_list(pairs)};
}

template <size_t Size, typename Key, typename Value>
constexpr Result<FixedMap<Key, Value, Size>>
fixed_map(const std::array<map_element<Key, Value>, Size> &pairs) {
  for (size_t i = 1; i < Size; ++i) {
    if (pairs[i].key == pairs[i - 1].key)
      return psm::Error::duplicate_key;
  }
  return FixedMap<Key, Value, Size>{std::initializer_list(pairs)};
}

template <size_t Size, typename Key, typename Value>
constexpr Result<FixedMap<Key, Value, Size>>
fixed_map(const FixedVec<map_element<Key, Value>, Size> &pairs) {
  for (size_t i = 1; i < Size; ++i) {
    if (pairs[i].key == pairs[i - 1].key)
      return psm::Error::duplicate_key;
  }
  return FixedMap<Key, Value, Size>{std::initializer_list(pairs)};
}
template <size_t Size, typename Key, typename Value>
constexpr Result<FixedMap<Key, Value, Size>>
fixed_map(FixedVec<map_element<Key, Value>, Size> &&pairs) {
  for (size_t i = 1; i < Size; ++i) {
    if (pairs[i].key == pairs[i - 1].key)
      return psm::Error::duplicate_key;
  }
  return FixedMap<Key, Value, Size>{std::move(pairs)};
}
/// @}

/**
 * create a fixed map from an initializer of map elements. If pairs contains
 * duplicate keys, an erroneous result is returned. The Key and Value are
 * deduced. Size must be specified as the first and only template parameter.
 */
template <size_t Size, typename Key, typename Value>
constexpr Result<FixedMap<Key, Value, Size>>
fixed_map(std::initializer_list<map_element<Key, Value>> il) {
  auto prev = il.begin();
  auto next = prev + 1;
  while (next != il.end()) {
    if (prev->key == next->key) {
      return psm::Error::duplicate_key;
    }
    ++prev;
    ++next;
  }
  return FixedMap<Key, Value, Size>{il};
}
} // namespace psm
#endif /* MAP_HPP_ */
