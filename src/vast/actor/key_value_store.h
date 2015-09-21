#ifndef VAST_ACTOR_KEY_VALUE_STORE_H
#define VAST_ACTOR_KEY_VALUE_STORE_H

#include <set>

#include "vast/filesystem.h"
#include "vast/none.h"
#include "vast/actor/basic_state.h"
#include "vast/util/radix_tree.h"

namespace vast {

/// A replicated hierarchical key-value store.
struct key_value_store {
  using storage = util::radix_tree<message>;

  struct state : basic_state {
    state(local_actor* self);

    storage data;
    util::radix_tree<none> persistent;
    actor leader;
    std::set<actor> followers;
  };

  /// Spawns a key-value store.
  /// @param self The actor handle.
  /// @param dir The directory used for persistence. If empty, the instance
  ///            operates in-memory only.
  static behavior make(stateful_actor<state>* self, path dir);
};

} // namespace vast

#endif
