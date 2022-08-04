//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2021 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "vast/as_bytes.hpp"
#include "vast/detail/byte_swap.hpp"
#include "vast/error.hpp"
#include "vast/logger.hpp"

#include <caf/detail/ieee_754.hpp>
#include <caf/detail/network_order.hpp>
#include <caf/detail/type_traits.hpp>
// #include <caf/detail/uri_impl.hpp>
#include <caf/expected.hpp>
#include <caf/load_inspector.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iterator>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>

namespace caf::legacy {
template <class Inspector, class T>
class is_inspectable {
private:
  template <class U>
  static auto sfinae(Inspector& x, U& y) -> decltype(inspect(x, y));

  static std::false_type sfinae(Inspector&, ...);

  using result_type
    = decltype(sfinae(std::declval<Inspector&>(), std::declval<T&>()));

public:
  static constexpr bool value
    = !std::is_same<result_type, std::false_type>::value;
};

template <class Inspector, class T>
struct is_inspectable<Inspector, T*> : std::false_type {};

/// Checks whether `T` provides either a free function or a member function for
/// serialization. The checks test whether both serialization and
/// deserialization can succeed. The meta function tests the following
/// functions with `Processor` being both `serializer` and `deserializer` and
/// returns an integral constant if and only if the test succeeds for both.
///
/// - `serialize(Processor&, T&, const unsigned int)`
/// - `serialize(Processor&, T&)`
/// - `T::serialize(Processor&, const unsigned int)`.
/// - `T::serialize(Processor&)`.
template <class T, bool Ignore
                   = std::is_pointer<T>::value || std::is_function<T>::value>
struct has_serialize {
  template <class U>
  static auto test_serialize(caf::serializer* sink, U* x, unsigned int y = 0)
    -> decltype(serialize(*sink, *x, y));

  template <class U>
  static auto test_serialize(caf::serializer* sink, U* x)
    -> decltype(serialize(*sink, *x));

  template <class>
  static auto test_serialize(...) -> std::false_type;

  template <class U>
  static auto
  test_deserialize(caf::deserializer* source, U* x, unsigned int y = 0)
    -> decltype(serialize(*source, *x, y));

  template <class U>
  static auto test_deserialize(caf::deserializer* source, U* x)
    -> decltype(serialize(*source, *x));

  template <class>
  static auto test_deserialize(...) -> std::false_type;

  using serialize_type = decltype(test_serialize<T>(nullptr, nullptr));
  using deserialize_type = decltype(test_deserialize<T>(nullptr, nullptr));
  using type
    = std::integral_constant<bool,
                             std::is_same<serialize_type, void>::value
                               && std::is_same<deserialize_type, void>::value>;

  static constexpr bool value = type::value;
};

template <class T>
struct has_serialize<T, true> {
  static constexpr bool value = false;
};

/// Any inspectable type is considered to be serializable.
template <class T>
struct is_serializable;

template <class T, bool IsIterable = caf::detail::is_iterable<T>::value,
          bool Ignore = std::is_pointer<T>::value || std::is_function<T>::value>
struct is_serializable_impl;

template <class T>
struct is_builtin {
  static constexpr bool value
    = std::is_arithmetic<T>::value || caf::detail::is_duration<T>::value
      || caf::detail::is_one_of<T, timestamp, std::string, std::u16string,
                                std::u32string, message, actor, group,
                                node_id>::value;
};

/// Checks whether `T` is builtin or provides a `serialize`
/// (free or member) function.
template <class T>
struct is_serializable_impl<T, false, false> {
  static constexpr bool value = has_serialize<T>::value
                                || is_inspectable<serializer, T>::value
                                || is_builtin<T>::value;
};

template <class Rep, class Period>
struct is_serializable_impl<std::chrono::duration<Rep, Period>, false, false> {
  static constexpr bool value = is_serializable<Rep>::value;
};

template <class Clock, class Duration>
struct is_serializable_impl<std::chrono::time_point<Clock, Duration>, false,
                            false> {
  static constexpr bool value = is_serializable<Duration>::value;
};

template <class F, class S>
struct is_serializable_impl<std::pair<F, S>, false, false> {
  static constexpr bool value
    = is_serializable<F>::value && is_serializable<S>::value;
};

template <class... Ts>
struct is_serializable_impl<std::tuple<Ts...>, false, false> {
  static constexpr bool value
    = std::conjunction_v<is_serializable<Ts>::value...>;
};

template <class T>
struct is_serializable_impl<T, true, false> {
  using value_type = typename T::value_type;
  static constexpr bool value = is_serializable<value_type>::value;
};

template <class T, size_t S>
struct is_serializable_impl<T[S], false, false> {
  static constexpr bool value = is_serializable<T>::value;
};

template <class T, bool IsIterable>
struct is_serializable_impl<T, IsIterable, true> {
  static constexpr bool value = false;
};

/// Checks whether `T` is builtin or provides a `serialize`
/// (free or member) function.
template <class T>
struct is_serializable {
  static constexpr bool value
    = is_serializable_impl<T>::value || is_inspectable<serializer, T>::value
      || std::is_empty<T>::value || std::is_enum<T>::value;
};

template <>
struct is_serializable<bool> : std::true_type {
  // nop
};

template <class T>
struct is_serializable<T&> : is_serializable<T> {
  // nop
};

template <class T>
struct is_serializable<const T> : is_serializable<T> {
  // nop
};

template <class T>
struct is_serializable<const T&> : is_serializable<T> {
  // nop
};
} // namespace caf::legacy

namespace vast::detail {

template <class T>
struct is_byte_sequence : std::false_type {};

template <>
struct is_byte_sequence<std::vector<char>> : std::true_type {};

template <>
struct is_byte_sequence<std::vector<unsigned char>> : std::true_type {};

template <>
struct is_byte_sequence<std::string> : std::true_type {};

template <int, bool>
struct select_integer_type;

template <>
struct select_integer_type<1, true> {
  using type = int8_t;
};

template <>
struct select_integer_type<1, false> {
  using type = uint8_t;
};

template <>
struct select_integer_type<2, true> {
  using type = int16_t;
};

template <>
struct select_integer_type<2, false> {
  using type = uint16_t;
};

template <>
struct select_integer_type<4, true> {
  using type = int32_t;
};

template <>
struct select_integer_type<4, false> {
  using type = uint32_t;
};

template <>
struct select_integer_type<8, true> {
  using type = int64_t;
};

template <>
struct select_integer_type<8, false> {
  using type = uint64_t;
};

template <int Size, bool IsSigned>
using select_integer_type_t =
  typename select_integer_type<Size, IsSigned>::type;

/// An inspector for CAF inspect
class legacy_deserializer {
public:
  static constexpr bool is_loading = false;
  using result_type = bool;

  explicit legacy_deserializer(std::span<const std::byte> bytes)
    : bytes_(bytes) {
  }

  template <class... Ts>
  result_type operator()(Ts&&... xs) noexcept {
    return (apply(xs) && ...);
  }

  template <class T>
  auto object(const T&) {
    return object_impl{*this};
  }

  template <class T>
  auto field(std::string_view, T& value) {
    return field_impl{value};
  }

  inline result_type apply_raw(size_t num_bytes, void* storage) {
    if (num_bytes > bytes_.size())
      return false;
    memcpy(storage, bytes_.data(), num_bytes);
    bytes_ = bytes_.subspan(num_bytes);
    return true;
  }

  template <class T>
    requires(caf::legacy::is_inspectable<legacy_deserializer, T>::value)
  result_type apply(T& x) {
    return inspect(*this, x);
  }

  // todo accept generic callback
  result_type apply(std::function<bool()> x) {
    return x();
  }

  template <class T>
    requires(std::is_enum_v<T>)
  result_type apply(T& x) {
    using underlying = typename std::underlying_type_t<T>;
    underlying tmp;
    if (!apply(tmp))
      return false;
    x = static_cast<T>(tmp);
    return true;
  }

  template <class F, class S>
    // todo some msg?
    requires(
      caf::legacy::is_serializable<typename std::remove_const_t<F>>::value&&
        caf::legacy::is_serializable<S>::value)
  result_type apply(std::pair<F, S>& xs) {
    using t0 = typename std::remove_const_t<F>;
    if (!apply(const_cast<t0&>(xs.first)))
      return false;
    return apply(xs.second);
  }

  // inline result_type apply(caf::uri& x) {
  //   auto impl = caf::make_counted<caf::detail::uri_impl>();
  //   if (!apply(*impl))
  //     return false;
  //   x = caf::uri{std::move(impl)};
  //   return true;
  // }

  inline result_type apply(bool& x) {
    uint8_t tmp = 0;
    if (!apply(tmp))
      return false;
    x = tmp != 0;
    return true;
  }

  inline result_type apply(int8_t& x) {
    return apply_raw(sizeof(x), &x);
  }

  inline result_type apply(uint8_t& x) {
    return apply_raw(sizeof(x), &x);
  }

  inline result_type apply(int16_t& x) {
    return apply_int(x);
  }

  inline result_type apply(uint16_t& x) {
    return apply_int(x);
  }

  inline result_type apply(int32_t& x) {
    return apply_int(x);
  }

  inline result_type apply(uint32_t& x) {
    return apply_int(x);
  }

  inline result_type apply(int64_t& x) {
    return apply_int(x);
  }

  inline result_type apply(uint64_t& x) {
    return apply_int(x);
  }

  template <class T>
    requires(std::is_integral_v<T> && !std::is_same_v<bool, T>)
  result_type apply(T& x) {
    using type = detail::select_integer_type_t<sizeof(T), std::is_signed_v<T>>;
    return apply(reinterpret_cast<type&>(x));
  }

  inline result_type apply(float& x) {
    return apply_float(x);
  }

  inline result_type apply(double& x) {
    return apply_float(x);
  }

  inline result_type apply(long double& x) {
    // The IEEE-754 conversion does not work for long double
    // => fall back to string serialization (even though it sucks).
    std::string tmp;
    if (!apply(tmp))
      return false;
    std::istringstream iss{std::move(tmp)};
    iss >> x;
    return true;
  }

  inline result_type apply(std::string& x) {
    size_t str_size = 0;
    if (!begin_sequence(str_size))
      return false;
    if (str_size > bytes_.size())
      return false;
    x.assign(reinterpret_cast<const char*>(bytes_.data()), str_size);
    bytes_ = bytes_.subspan(str_size);
    return true;
  }

  inline result_type apply(caf::none_t& x) {
    x = caf::none;
    return true;
  }

  template <class Rep, class Period>
    requires(std::is_integral_v<Rep>)
  result_type apply(std::chrono::duration<Rep, Period>& x) {
    using duration_type = std::chrono::duration<Rep, Period>;
    Rep tmp;
    if (!apply(tmp))
      return false;
    x = duration_type{tmp};
    return true;
  }

  template <class Rep, class Period>
    requires(std::is_floating_point_v<Rep>)
  result_type apply(std::chrono::duration<Rep, Period>& x) {
    using duration_type = std::chrono::duration<Rep, Period>;
    // always save/store floating point durations as doubles
    double tmp = NAN;
    if (!apply(tmp))
      return false;
    x = duration_type{tmp};
    return true;
  }

  template <class T>
    requires(caf::detail::is_iterable<T>::value
             && !caf::legacy::is_inspectable<legacy_deserializer, T&>::value)
  result_type apply(T& xs) {
    return apply_sequence(xs);
  }

  template <class Clock, class Duration>
  result_type apply(std::chrono::time_point<Clock, Duration>& t) {
    Duration dur{};
    if (!apply(dur))
      return false;
    t = std::chrono::time_point<Clock, Duration>{dur};
    return true;
  }

  template <class T>
    requires(!detail::is_byte_sequence<T>::value)
  result_type apply_sequence(T& xs) {
    size_t size = 0;
    if (!begin_sequence(size))
      return false;
    xs.clear();
    auto it = std::inserter(xs, xs.end());
    for (size_t i = 0; i < size; ++i) {
      typename T::value_type tmp;
      if (!apply(tmp))
        return false;
      *it++ = std::move(tmp);
    }
    return true;
  }

  template <class T, std::size_t N>
  result_type apply(std::array<T, N>& x) {
    for (T& v : x)
      if (!apply(v))
        return false;
    return true;
  }

private:
  class object_impl {
  public:
    explicit object_impl(legacy_deserializer& self) : self_{self} {
    }

    template <class... Fields>
    bool fields(Fields&&... fields) {
      auto success = (fields(self_) && ...);
      if (success && load_callback_)
        success = load_callback_();
      return success;
    }

    object_impl& pretty_name(std::string_view) {
      return *this;
    }

    template <class Callback>
    object_impl& on_load(Callback callback) {
      load_callback_ = std::move(callback);
      return *this;
    }

  private:
    legacy_deserializer& self_;
    std::function<bool()> load_callback_;
  };

  template <class T>
  class field_impl {
  public:
    explicit field_impl(T& value) : value_{value} {
    }
    bool operator()(legacy_deserializer& f) {
      return f.apply(value_);
    }

  private:
    T& value_;
  };

  inline result_type begin_sequence(size_t& list_size) {
    // Use varbyte encoding to compress sequence size on the wire.
    uint32_t x = 0;
    int n = 0;
    uint8_t low7 = 0;
    do {
      if (!apply(low7))
        return false;
      x |= static_cast<uint32_t>((low7 & 0x7F)) << (7 * n);
      ++n;
    } while ((low7 & 0x80) != 0);
    list_size = x;
    return true;
  }

  template <class T>
  result_type apply_float(T& x) {
    typename caf::detail::ieee_754_trait<T>::packed_type tmp;
    if (!apply_int(tmp))
      return false;
    x = caf::detail::unpack754(tmp);
    return true;
  }

  template <class T>
  result_type apply_int(T& x) {
    std::make_unsigned_t<T> tmp;
    if (!apply_raw(sizeof(x), &tmp))
      return false;
    x = static_cast<T>(to_host_order(tmp));
    return true;
  }

  std::span<const std::byte> bytes_ = {};
};

/// Deserializes a sequence of objects from a byte buffer.
/// @param buffer The vector of bytes to read from.
/// @param xs The object to deserialize.
/// @returns The status of the operation.
/// @relates detail::serialize
template <concepts::byte_container Buffer, class... Ts>
  requires(!std::is_rvalue_reference_v<Ts> && ...)
bool legacy_deserialize(const Buffer& buffer, Ts&... xs) {
  legacy_deserializer f{as_bytes(buffer)};
  return f(xs...);
}

} // namespace vast::detail
