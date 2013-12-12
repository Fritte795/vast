#ifndef VAST_PROGRAM_H
#define VAST_PROGRAM_H

#include <cppa/cppa.hpp>
#include "vast/actor.h"
#include "vast/config.h"
#include "vast/configuration.h"

namespace vast {

/// The main program.
class program : public actor<program>
{
  program(program const&) = delete;
  program& operator=(program) = delete;

public:
  /// Spawns the program.
  /// @param config The program configuration.
  program(configuration const& config);

  void act();
  char const* description() const;

private:
  configuration const& config_;

  cppa::actor_ptr receiver_;
  cppa::actor_ptr archive_;
  cppa::actor_ptr index_;
  cppa::actor_ptr tracker_;
  cppa::actor_ptr ingestor_;
  cppa::actor_ptr search_;
  cppa::actor_ptr schema_manager_;
  cppa::actor_ptr signal_monitor_;
  cppa::actor_ptr profiler_;

#ifdef VAST_HAVE_EDITLINE
  cppa::actor_ptr console_;
  bool server_ = true;
#endif
};

} // namespace vast

#endif
