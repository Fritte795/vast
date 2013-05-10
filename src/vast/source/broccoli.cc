#include "vast/source/broccoli.h"

#include "vast/logger.h"
#include "vast/util/broccoli.h"

namespace vast {
namespace source {

using namespace cppa;

broccoli::broccoli(std::string const& host, unsigned port)
{
  LOG(verbose, core) << "spawning broccoli source @" << id();
  operating_ = (
      on(atom("kill")) >> [=]
      {
        server_ << last_dequeued();
        quit();
        LOG(verbose, ingest) << "broccoli source @" << id() << " terminated";
      },
      on(atom("run")) >> [=]
      {
        // TODO: Make use of the host argument.
        LOG(verbose, core) << "broccoli @" << id()
          << "starts server at " << host << ':' << port;
        server_ = spawn<util::broccoli::server>(port, self);
        monitor(server_);
      },
      on(atom("DOWN"), arg_match) >> [](size_t exit_reason)
      {
        LOG(error, ingest)
          << "broccoli source @" << id() 
          << " noticed termination of its server @" << server_->id();
        send(self, atom("shutdown"));
      },
      on(atom("connection"), arg_match) >> [=](actor_ptr conn)
      {
        for (auto& event : event_names_)
          send(conn, atom("subscribe"), event);
        send(conn, atom("start"), self);
      },
      on(atom("subscribe"), arg_match) >> [=](std::string const& event)
      {
        LOG(verbose, ingest)
          << "broccoli source @" << id() << " subscribes to event " << event;
        event_names_.insert(event);
      },
      on(atom("subscribe"), arg_match)
        >> [=](std::vector<std::string> const& events)
      {
        for (auto& e : events)
          send(self, atom("subscribe"), e);
      });
}

} // namespace source
} // namespace vast
