//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2020 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "vast/fwd.hpp"

#include "vast/detail/inspection_common.hpp"

#include <cstdint>

namespace vast::system {

/// Statistics about filesystem operations.
struct filesystem_statistics {
  struct ops {
    uint64_t successful = 0;
    uint64_t failed = 0;
    uint64_t bytes = 0;

    template <class Inspector>
    friend auto inspect(Inspector& f, ops& x) {
      f.object(x).pretty_name("vast.system.filesystem_statistics.ops");
      return detail::inspect(x.successful, x.failed, x.bytes);
    }
  };

  ops checks;
  ops writes;
  ops reads;
  ops mmaps;
  ops erases;
  ops moves;

  template <class Inspector>
  friend auto inspect(Inspector& f, filesystem_statistics& x) {
    f.object(x).pretty_name("vast.system.filesystem_statistics");
    return detail::inspect(x.checks, x.writes, x.reads, x.mmaps, x.moves);
  }
};

} // namespace vast::system
