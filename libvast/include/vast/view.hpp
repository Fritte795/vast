//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2018 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "vast/fwd.hpp"

#include "vast/aliases.hpp"
#include "vast/data.hpp"
#include "vast/detail/assert.hpp"
#include "vast/detail/iterator.hpp"
#include "vast/detail/operators.hpp"
#include "vast/detail/type_traits.hpp"
#include "vast/hash/hash.hpp"
#include "vast/time.hpp"

#include <caf/intrusive_ptr.hpp>
#include <caf/make_counted.hpp>
#include <caf/optional.hpp>
#include <caf/ref_counted.hpp>
#include <caf/variant.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <type_traits>
#include <typeindex>

namespace vast {

/// A type-safe overlay over an immutable sequence of bytes.
template <class>
struct view_trait;

/// @relates view_trait
template <class T>
using view = typename view_trait<T>::type;

#define VAST_VIEW_TRAIT(type_name)                                             \
  template <>                                                                  \
  struct view_trait<type_name> {                                               \
    using type = type_name;                                                    \
  };                                                                           \
  inline type_name materialize(view<type_name> x) {                            \
    return x;                                                                  \
  }

VAST_VIEW_TRAIT(bool)
VAST_VIEW_TRAIT(integer)
VAST_VIEW_TRAIT(count)
VAST_VIEW_TRAIT(real)
VAST_VIEW_TRAIT(duration)
VAST_VIEW_TRAIT(time)
VAST_VIEW_TRAIT(enumeration)
VAST_VIEW_TRAIT(address)
VAST_VIEW_TRAIT(subnet)

#undef VAST_VIEW_TRAIT

/// @relates view_trait
template <>
struct view_trait<caf::none_t> {
  using type = caf::none_t;
};

/// @relates view_trait
template <>
struct view_trait<std::string> {
  using type = std::string_view;
};

/// @relates view_trait
class pattern_view : detail::totally_ordered<pattern_view> {
public:
  static pattern glob(std::string_view x);

  explicit pattern_view(const pattern& x);

  explicit pattern_view(std::string_view str);

  [[nodiscard]] bool match(std::string_view x) const;
  [[nodiscard]] bool search(std::string_view x) const;
  [[nodiscard]] std::string_view string() const;

  template <class Hasher>
  friend void hash_append(Hasher& h, pattern_view x) {
    hash_append(h, x.pattern_);
  }

private:
  std::string_view pattern_;
};

/// @relates pattern_view
bool operator==(pattern_view x, pattern_view y) noexcept;

/// @relates pattern_view
bool operator<(pattern_view x, pattern_view y) noexcept;

//// @relates view_trait
template <>
struct view_trait<pattern> {
  using type = pattern_view;
};

template <class T>
class container_view_handle;

struct list_view_ptr;
using list_view_handle = container_view_handle<list_view_ptr>;

/// @relates view_trait
template <>
struct view_trait<list> {
  using type = list_view_handle;
};

struct map_view_ptr;
using map_view_handle = container_view_handle<map_view_ptr>;

/// @relates view_trait
template <>
struct view_trait<map> {
  using type = map_view_handle;
};

struct record_view_ptr;
using record_view_handle = container_view_handle<record_view_ptr>;

/// @relates view_trait
template <>
struct view_trait<record> {
  using type = record_view_handle;
};

// clang-format off
/// A type-erased view over various types of data.
/// @relates view_trait
using data_view = caf::variant<
  view<caf::none_t>,
  view<bool>,
  view<integer>,
  view<count>,
  view<real>,
  view<duration>,
  view<time>,
  view<std::string>,
  view<pattern>,
  view<address>,
  view<subnet>,
  view<enumeration>,
  view<list>,
  view<map>,
  view<record>
>;
// clang-format on

/// @relates view_trait
template <>
struct view_trait<data> {
  using type = data_view;
};

// -- operators ----------------------------------------------------------------

// We cannot use operator== and operator!= here because data has a non-explicit
// constructor, which results in error all over the code base. Therefore, we
// work around this by giving this function a name, and calling it from a
// templated friend operator== and operator!= in data.

bool is_equal(const data& x, const data_view& y);

bool is_equal(const data_view& x, const data& y);

// -- containers ---------------------------------------------------------------

template <class T>
struct container_view;

/// @relates view_trait
template <class T>
using container_view_ptr = caf::intrusive_ptr<container_view<T>>;

/// @relates container_view
template <class Pointer>
class container_view_handle
  : detail::totally_ordered<container_view_handle<Pointer>> {
public:
  using view_type = typename Pointer::element_type;

  using iterator = typename view_type::iterator;

  container_view_handle() = default;

  explicit container_view_handle(Pointer ptr) : ptr_{ptr} {
    // nop
  }

  explicit operator bool() const {
    return static_cast<bool>(ptr_);
  }

  auto operator->() const {
    return ptr_.get();
  }

  const auto& operator*() const {
    return *ptr_;
  }

  [[nodiscard]] iterator begin() const {
    return {ptr_.get(), 0};
  }

  [[nodiscard]] iterator end() const {
    return {ptr_.get(), size()};
  }

  [[nodiscard]] size_t size() const {
    return ptr_ ? ptr_->size() : 0;
  }

  [[nodiscard]] bool empty() const {
    return size() == 0;
  }

  template <class Hasher>
  friend void hash_append(Hasher& h, container_view_handle xs) {
    // TODO: include the concrete view type in the hash digest so that it
    // guarantees the absense of collisions between view types.
    if (!xs)
      return hash_append(h, caf::none);
    for (auto x : *xs)
      hash_append(h, x);
    hash_append(h, xs->size());
  }

  friend bool
  operator==(const container_view_handle& x, const container_view_handle& y) {
    return x && y && *x == *y;
  }

  friend bool
  operator<(const container_view_handle& x, const container_view_handle& y) {
    if (!x)
      return static_cast<bool>(y);
    if (!y)
      return false;
    return *x < *y;
  }

private:
  Pointer ptr_;
};

namespace detail {

/// @relates view_trait
template <class T>
class container_view_iterator
  : public detail::iterator_facade<container_view_iterator<T>, T,
                                   std::random_access_iterator_tag, T> {
  friend iterator_access;

public:
  container_view_iterator(typename container_view_ptr<T>::const_pointer ptr,
                          size_t pos)
    : view_{ptr}, position_{pos} {
    // nop
  }

  [[nodiscard]] auto dereference() const {
    return view_->at(position_);
  }

  void increment() {
    ++position_;
  }

  void decrement() {
    --position_;
  }

  template <class N>
  void advance(N n) {
    position_ += n;
  }

  [[nodiscard]] bool equals(container_view_iterator other) const {
    return view_ == other.view_ && position_ == other.position_;
  }

  [[nodiscard]] auto distance_to(container_view_iterator other) const {
    return other.position_ - position_;
  }

private:
  typename container_view_ptr<T>::const_pointer view_;
  size_t position_;
};

} // namespace detail

/// Base class for container views.
/// @relates view_trait
template <class T>
struct container_view : caf::ref_counted,
                        detail::totally_ordered<container_view<T>> {
  using value_type = T;
  using size_type = size_t;
  using iterator = detail::container_view_iterator<T>;
  using const_iterator = iterator;

  ~container_view() override = default;

  iterator begin() const {
    return {this, 0};
  }

  iterator end() const {
    return {this, size()};
  }

  /// Retrieves a specific element.
  /// @param i The position of the element to retrieve.
  /// @returns A view to the element at position *i*.
  /// @pre `i < size()`
  virtual value_type at(size_type i) const = 0;

  /// @returns The number of elements in the container.
  virtual size_type size() const noexcept = 0;

  /// @returns `true` if the container is empty.
  bool empty() const noexcept {
    return size() == 0;
  }
};

template <class T>
bool operator==(const container_view<T>& xs, const container_view<T>& ys) {
  if (xs.size() != ys.size())
    return false;
  for (auto i = 0u; i < xs.size(); ++i)
    if (xs.at(i) != ys.at(i))
      return false;
  return true;
}

template <class T>
bool operator<(const container_view<T>& xs, const container_view<T>& ys) {
  if (xs.size() != ys.size())
    return xs.size() < ys.size();
  for (auto i = 0u; i < xs.size(); ++i)
    if (xs.at(i) < ys.at(i))
      return true;
  return false;
}

// @relates view_trait
struct list_view_ptr : container_view_ptr<data_view> {};

/// A view over a @ref list.
/// @relates view_trait
class default_list_view : public container_view<data_view>,
                          detail::totally_ordered<default_list_view> {
public:
  explicit default_list_view(const list& xs);

  value_type at(size_type i) const override;

  size_type size() const noexcept override;

private:
  const list& xs_;
};

// @relates view_trait
struct map_view_ptr : container_view_ptr<std::pair<data_view, data_view>> {};

/// A view over a @ref map.
/// @relates view_trait
class default_map_view : public container_view<std::pair<data_view, data_view>>,
                         detail::totally_ordered<default_map_view> {
public:
  explicit default_map_view(const map& xs);

  value_type at(size_type i) const override;

  size_type size() const noexcept override;

private:
  const map& xs_;
};

// @relates view_trait
struct record_view_ptr
  : container_view_ptr<std::pair<std::string_view, data_view>> {};

/// A view over a @ref record.
/// @relates view_trait
class default_record_view
  : public container_view<std::pair<std::string_view, data_view>>,
    detail::totally_ordered<default_record_view> {
public:
  explicit default_record_view(const record& xs);

  value_type at(size_type i) const override;

  size_type size() const noexcept override;

private:
  const record& xs_;
};

// -- factories ----------------------------------------------------------------

/// Creates a view from a specific type.
/// @relates view_trait
template <class T>
view<T> make_view(const T& x) {
  constexpr auto directly_constructible
    = detail::is_any_v<T, caf::none_t, bool, integer, count, real, duration,
                       time, std::string, address, subnet, enumeration>;
  if constexpr (directly_constructible) {
    return x;
  } else if constexpr (std::is_same_v<T, pattern>) {
    return pattern_view{x};
  } else if constexpr (std::is_same_v<T, list>) {
    return list_view_handle{
      list_view_ptr{caf::make_counted<default_list_view>(x)}};
  } else if constexpr (std::is_same_v<T, map>) {
    return map_view_handle{
      map_view_ptr{caf::make_counted<default_map_view>(x)}};
  } else if constexpr (std::is_same_v<T, record>) {
    return record_view_handle{
      record_view_ptr{caf::make_counted<default_record_view>(x)}};
  } else {
    VAST_ASSERT(!"missing branch");
    return {};
  }
}

/// Creates a view from a string literal.
/// @relates view_trait
template <size_t N>
view<std::string> make_view(const char (&xs)[N]) {
  return std::string_view(xs, N - 1);
}

/// Creates a view from a `std::string_view`.
/// @relates view_trait
constexpr view<std::string> make_view(std::string_view xs) {
  return xs;
}

/// @relates view data
data_view make_view(const data& x);

/// Creates a type-erased data view from a specific type.
/// @relates view_trait
template <class T>
data_view make_data_view(const T& x) {
  return make_view(x);
}

/// @relates view_trait
template <class T>
data_view make_data_view(const caf::optional<T>& x) {
  if (!x)
    return make_view(caf::none);
  return make_view(*x);
}

// -- materialization ----------------------------------------------------------

constexpr auto materialize(caf::none_t x) {
  return x;
}

std::string materialize(std::string_view x);

pattern materialize(pattern_view x);

list materialize(list_view_handle xs);

map materialize(map_view_handle xs);

record materialize(record_view_handle xs);

data materialize(data_view xs);

// -- utilities ----------------------------------------------------------------

/// Checks whether data is valid for a given type.
/// @param x The type that describes *x*.
/// @param y The data view to be checked against *x*.
/// @returns `true` if *x* is a valid type for *y*.
bool type_check(const type& x, const data_view& y);

/// Converts a value from its internal representation to the type used in the
/// user interface. This is the inverse of to_internal.
/// @param t The type that describes *x*.
/// @param x The data view on the internal representation of the value.
/// @return A data view on the external representation of the value.
data_view to_canonical(const type& t, const data_view& x);

/// Converts a value from the type defined in the user interface to its
/// internal representation. This is the inverse of to_canonical.
/// @param t The type that describes *x*.
/// @param x The data view on the external representation of the value.
/// @return A data view on the internal representation of the value.
data_view to_internal(const type& t, const data_view& x);

} // namespace vast

namespace std {

template <>
struct hash<vast::data_view> {
  auto operator()(const vast::data_view& x) const {
    return vast::hash(x);
  }

  auto operator()(const vast::data& x) const {
    // The hash computation for `data` and `data_view` is subtly different when
    // not creating a view here: For `data_view` a hash of the contents of the
    // view is created (so that the same data compares equal whether it's
    // stored in a default/msgpack/arrow view), but for `data` the hashing is
    // forwarded to the actual container classes, which can define their own
    // hash functions.
    return (*this)(vast::make_view(x));
  }
};

template <>
struct hash<vast::data> : hash<vast::data_view> {};

template <>
struct equal_to<vast::data_view> {
  using is_transparent = void; // Opt-in to heterogeneous lookups.

  template <class Lhs, class Rhs>
  constexpr bool operator()(const Lhs& lhs, const Rhs& rhs) const
    noexcept(noexcept(lhs == rhs)) {
    static_assert(vast::detail::is_any_v<Lhs, vast::data, vast::data_view>);
    static_assert(vast::detail::is_any_v<Rhs, vast::data, vast::data_view>);
    return lhs == rhs;
  }
};

template <>
struct equal_to<vast::data> : equal_to<vast::data_view> {};

} // namespace std

#include "vast/concept/printable/vast/view.hpp"

namespace fmt {

template <>
struct formatter<vast::data_view> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const vast::data_view& value, FormatContext& ctx) const {
    auto out = ctx.out();
    vast::print(out, value);
    return out;
  }
};

} // namespace fmt
