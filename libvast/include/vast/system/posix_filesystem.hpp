//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2020 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "vast/fwd.hpp"

#include "vast/system/actors.hpp"
#include "vast/system/filesystem_statistics.hpp"

#include <caf/typed_event_based_actor.hpp>

#include <filesystem>

namespace vast::system {

/// The state for the POSIX filesystem.
/// @relates posix_filesystem
struct posix_filesystem_state {
  /// Statistics about filesystem operations.
  filesystem_statistics stats;

  /// The filesystem root.
  std::filesystem::path root;

  /// The actor name.
  static inline const char* name = "posix-filesystem";

  // Rename a file and update statistics.
  caf::expected<atom::done> rename_single_file(const std::filesystem::path&,
                                               const std::filesystem::path&);
};

/// A filesystem implemented with POSIX system calls.
/// @param self The actor handle.
/// @param root The filesystem root. The actor prepends this path to all
///             operations that include a path parameter.
/// @returns The actor behavior.
filesystem_actor::behavior_type
posix_filesystem(filesystem_actor::stateful_pointer<posix_filesystem_state> self,
                 const std::filesystem::path& root);

} // namespace vast::system
