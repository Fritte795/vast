//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2021 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "broker/zeek.hpp"

#include <vast/address.hpp>
#include <vast/as_bytes.hpp>
#include <vast/concept/printable/to_string.hpp>
#include <vast/concept/printable/vast/address.hpp>
#include <vast/detail/byte_swap.hpp>
#include <vast/detail/narrow.hpp>
#include <vast/detail/type_traits.hpp>
#include <vast/error.hpp>
#include <vast/logger.hpp>

namespace vast::plugins::broker {

// Things we take directly from zeek.
namespace zeek {

enum class tag : int {
  type_void,     // 0
  type_bool,     // 1
  type_int,      // 2
  type_count,    // 3
  type_counter,  // 4
  type_double,   // 5
  type_time,     // 6
  type_interval, // 7
  type_string,   // 8
  type_pattern,  // 9
  type_enum,     // 10
  type_timer,    // 11
  type_port,     // 12
  type_addr,     // 13
  type_subnet,   // 14
  type_any,      // 15
  type_table,    // 16
  type_union,    // 17
  type_record,   // 18
  type_list,     // 19
  type_func,     // 20
  type_file,     // 21
  type_vector,   // 22
  type_opaque,   // 23
  type_type,     // 24
  type_error,    // 25
  MAX = type_error,
};

/// Parses a value out of binary Zeek data.
/// @param bytes The raw bytes to parse.
/// @returns An error on failure.
/// @post *bytes* is advanced by the number of bytes of the extract value.
template <class T>
caf::error extract(T& x, span<const std::byte>& bytes) {
  if constexpr (std::is_same_v<T, char>) {
    if (bytes.empty())
      return caf::make_error(ec::parse_error, "input exhausted");
    x = static_cast<char>(bytes[0]);
    bytes = bytes.subspan(1);
  } else if constexpr (std::is_same_v<T, bool>) {
    char c;
    if (auto err = extract(c, bytes))
      return err;
    x = (c == '\1');
  } else if constexpr (std::is_same_v<T, int>) {
    // In Zeek, an int has always 32 bits on the wire.
    uint32_t result;
    if (auto err = extract(result, bytes))
      return err;
    x = static_cast<int>(result);
  } else if constexpr (std::is_same_v<T, double>) {
    if (bytes.size() < sizeof(T))
      return caf::make_error(ec::parse_error, "input exhausted");
    std::memcpy(&x, bytes.data(), sizeof(T));
    // Directly lifted from src/net_util.h.
    static auto ntohd = [](double d) {
      VAST_ASSERT(sizeof(d) == 8);
      double tmp;
      char* src = (char*)&d;
      char* dst = (char*)&tmp;
      dst[0] = src[7];
      dst[1] = src[6];
      dst[2] = src[5];
      dst[3] = src[4];
      dst[4] = src[3];
      dst[5] = src[2];
      dst[6] = src[1];
      dst[7] = src[0];
      return tmp;
    };
    x = ntohd(x);
    bytes = bytes.subspan(sizeof(T));
  } else if constexpr (std::is_integral_v<T>) {
    std::make_unsigned_t<T> u;
    if (bytes.size() < sizeof(T))
      return caf::make_error(ec::parse_error, "input exhausted");
    std::memcpy(&u, bytes.data(), sizeof(T));
    x = detail::narrow_cast<T>(detail::to_host_order(u));
    bytes = bytes.subspan(sizeof(T));
  } else if constexpr (std::is_same_v<T, std::string>) {
    uint32_t length = 0;
    if (auto err = extract(length, bytes))
      return err;
    if (length > bytes.size())
      return caf::make_error(ec::parse_error, "input exhausted");
    x.resize(length);
    std::memcpy(x.data(), bytes.data(), x.size());
    bytes = bytes.subspan(x.size());
  } else if constexpr (std::is_same_v<T, address>) {
    char family;
    if (auto err = extract(family, bytes))
      return err;
    switch (family) {
      default:
        return caf::make_error(ec::parse_error, "invalid addr family", family);
      case 4: {
        if (bytes.size() < 4)
          return caf::make_error(ec::parse_error, "input exhausted");
        x = address::v4(bytes.data(), address::byte_order::network);
        bytes = bytes.subspan(4);
        break;
      }
      case 6:
        if (bytes.size() < 16)
          return caf::make_error(ec::parse_error, "input exhausted");
        x = address::v6(bytes.data(), address::byte_order::network);
        bytes = bytes.subspan(16);
        break;
    }
  }
  return {};
}

/// Parses a binary Zeek value.
/// TODO: add an output parameter or function similar to extract above.
caf::error extract_value(span<const std::byte>& bytes) {
  // Every value begins with type information.
  int type, sub_type;
  bool present;
  auto err = caf::error::eval(
    [&] {
      return extract(type, bytes);
    },
    [&] {
      return extract(sub_type, bytes);
    },
    [&] {
      return extract(present, bytes);
    });
  if (err)
    return err;
  // Skip null values.
  if (!present) {
    VAST_INFO("nil");
    return {};
  }
  // Dispatch on the Zeek tag type.
  switch (static_cast<zeek::tag>(type)) {
    default:
      return caf::make_error(ec::parse_error, "unsupported value type", type);
    case zeek::tag::type_bool: {
      int64_t x;
      if (auto err = extract(x, bytes))
        return err;
      VAST_INFO("bool = {}", !!x);
      break;
    }
    case zeek::tag::type_int: {
      int64_t x;
      if (auto err = extract(x, bytes))
        return err;
      VAST_INFO("int = {}", x);
      break;
    }
    case zeek::tag::type_count:
    case zeek::tag::type_counter: {
      uint64_t x;
      if (auto err = extract(x, bytes))
        return err;
      VAST_INFO("count = {}", x);
      break;
    }
    case zeek::tag::type_port: {
      uint64_t number;
      int proto;
      if (auto err = extract(number, bytes))
        return err;
      if (auto err = extract(proto, bytes))
        return err;
      VAST_INFO("port = {}/{}", number, proto);
      break;
    }
    case zeek::tag::type_addr: {
      address addr;
      if (auto err = extract(addr, bytes))
        return err;
      VAST_INFO("addr = {}", to_string(addr));
      break;
    }
    case zeek::tag::type_subnet: {
      uint8_t length;
      if (auto err = extract(length, bytes))
        return err;
      address addr;
      if (auto err = extract(addr, bytes))
        return err;
      VAST_INFO("subnet = {}/{}", addr, length);
      break;
    }
    case zeek::tag::type_double:
    case zeek::tag::type_time:
    case zeek::tag::type_interval: {
      double x;
      if (auto err = extract(x, bytes))
        return err;
      VAST_INFO("double = {}", x);
      break;
    }
    case zeek::tag::type_enum:
    case zeek::tag::type_string:
    case zeek::tag::type_file:
    case zeek::tag::type_func: {
      std::string x;
      if (auto err = extract(x, bytes))
        return err;
      VAST_INFO("string = {}", x);
      break;
    }
    case zeek::tag::type_table: {
      int64_t size;
      if (auto err = extract(size, bytes))
        return err;
      VAST_INFO("table of size {}", size);
      for (auto i = 0; i < size; ++i)
        if (auto err = extract_value(bytes))
          return err;
      break;
    }
    case zeek::tag::type_vector: {
      int64_t size;
      if (auto err = extract(size, bytes))
        return err;
      VAST_INFO("vector of size {}", size);
      for (auto i = 0; i < size; ++i)
        if (auto err = extract_value(bytes))
          return err;
      break;
    }
  }
  return {};
}

} // namespace zeek

caf::error process(const ::broker::zeek::LogWrite& msg) {
  auto serial_data = caf::get_if<std::string>(&msg.serial_data());
  if (!serial_data) {
    VAST_WARN("got invalid LogWrite serial data");
    return ec::parse_error;
  }
  auto bytes = as_bytes(*serial_data);
  // Read the number of fields.
  uint32_t num_fields;
  if (auto err = zeek::extract(num_fields, bytes))
    return err;
  // Read as many "threading values" as there are fields.
  for (size_t i = 0; i < num_fields; ++i) {
    if (auto err = zeek::extract_value(bytes))
      return err;
  }
  if (bytes.size() > 0)
    VAST_ERROR("incomplete write, {} remaining bytes", bytes.size());
  return {};
}

} // namespace vast::plugins::broker
