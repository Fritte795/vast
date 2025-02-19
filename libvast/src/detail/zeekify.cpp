//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2021 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "vast/detail/zeekify.hpp"

#include "vast/logger.hpp"

#include <array>
#include <string>

namespace vast::detail {

namespace {

// For fields that do not require substring search, use an optimized index.
bool is_opaque_id(const auto& field) {
  if (!caf::holds_alternative<string_type>(field.type))
    return false;
  auto has_name = [&](const auto& name) {
    return name == field.name;
  };
  // TODO: do more than this simple heuristic. For example, we should also
  // consider zeek.files.conn_uids, which is a set of strings. The inner
  // index needs to have the #index=hash tag. Moreover, we need to consider
  // other fields, such as zeek.x509.id instead of uid.
  static auto ids = std::array{"uid", "fuid", "community_id"};
  return std::find_if(ids.begin(), ids.end(), has_name) != ids.end();
}

} // namespace

record_type zeekify(record_type layout) {
  auto transformations = std::vector<record_type::transformation>{};
  transformations.reserve(layout.num_leaves());
  bool found_event_timestamp = false;
  for (const auto& [field, offset] : layout.leaves()) {
    if (!found_event_timestamp && field.name == "ts"
        && caf::holds_alternative<time_type>(field.type)) {
      // The first field is almost exclusively the event timestamp for standard
      // Zeek logs. Its has the field name `ts`. For streaming JSON, some other
      // fields, e.g., `_path`, precede it.
      VAST_DEBUG("using timestamp type for field {}", field.name);
      transformations.push_back({
        offset,
        record_type::assign({
          {
            "ts",
            type{"timestamp", time_type{}},
          },
        }),
      });
      found_event_timestamp = true;
    } else if (is_opaque_id(field)) {
      VAST_DEBUG("using hash index for field {}", field.name);
      transformations.push_back({
        offset,
        record_type::assign({
          {
            std::string{field.name},
            type{field.type, {{"index", "hash"}}},
          },
        }),
      });
    }
  }
  auto adjusted_layout = layout.transform(std::move(transformations));
  return adjusted_layout ? std::move(*adjusted_layout) : std::move(layout);
}

} // namespace vast::detail
