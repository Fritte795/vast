//    _   _____   __________
//   | | / / _ | / __/_  __/     Visibility
//   | |/ / __ |_\ \  / /          Across
//   |___/_/ |_/___/ /_/       Space and Time
//
// SPDX-FileCopyrightText: (c) 2022 The VAST Contributors
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace vast::detail {

template <class Inspector, class... Args>
bool inspect(Inspector& f, Args&&... args) {
  return (f.apply(std::forward<Args>(args)) && ...);
}

} // namespace vast::detail
