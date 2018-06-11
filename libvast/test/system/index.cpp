/******************************************************************************
 *                    _   _____   __________                                  *
 *                   | | / / _ | / __/_  __/     Visibility                   *
 *                   | |/ / __ |_\ \  / /          Across                     *
 *                   |___/_/ |_/___/ /_/       Space and Time                 *
 *                                                                            *
 * This file is part of VAST. It is subject to the license terms in the       *
 * LICENSE file found in the top-level directory of this distribution and at  *
 * http://vast.io/license. No part of VAST, including this file, may be       *
 * copied, modified, propagated, or distributed except according to the terms *
 * contained in the LICENSE file.                                             *
 ******************************************************************************/

#include "vast/system/index.hpp"

#define SUITE index
#include "test.hpp"

#include "vast/concept/parseable/to.hpp"
#include "vast/concept/parseable/vast/expression.hpp"
#include "vast/concept/printable/std/chrono.hpp"
#include "vast/concept/printable/to_string.hpp"
#include "vast/detail/spawn_generator_source.hpp"
#include "vast/event.hpp"
#include "vast/ids.hpp"
#include "vast/query_options.hpp"

#include "fixtures/actor_system_and_events.hpp"

using caf::after;
using std::chrono_literals::operator""s;

using namespace vast;
using namespace std::chrono;

namespace {

static constexpr size_t partition_size = 100;

static constexpr size_t in_mem_partitions = 8;

static constexpr size_t taste_count = 4;

static constexpr size_t num_collectors = 1;

const timestamp epoch;

using interval = system::partition_index::interval;

auto int_generator(int mod = std::numeric_limits<int>::max()) {
  int i = 0;
  return [i, mod]() mutable {
    auto result = event::make(i % mod, integer_type{}, i);
    result.timestamp(epoch + std::chrono::seconds(i));
    ++i;
    return result;
  };
}

struct fixture : fixtures::deterministic_actor_system_and_events {
  fixture() {
    directory /= "index";
    index = self->spawn(system::index, directory / "index", partition_size,
                        in_mem_partitions, taste_count, num_collectors);
  }

  ~fixture() {
    anon_send_exit(index, caf::exit_reason::user_shutdown);
  }

  // Returns the state of the `index`.
  system::index_state& state() {
    return deref<caf::stateful_actor<system::index_state>>(index).state;
  }

  auto partition_intervals() {
    std::vector<interval> result;
    for (auto& kvp : state().part_index.partitions())
      result.emplace_back(kvp.second.range);
    std::sort(result.begin(), result.end(),
              [](auto& x, auto& y) { return x.from < y.from; });
    return result;
  }

  auto receive_query_id() {
    std::tuple<uuid, size_t, size_t> result;
    self->receive(
      [&](uuid& query_id, size_t hits, size_t scheduled) {
        result = std::tie(query_id, hits, scheduled);
      },
      after(0s) >> [&] { FAIL("INDEX did not respond to query"); });
    return result;
  }

  ids receive_result(const uuid& query_id, size_t hits, size_t scheduled) {
    ids result;
    size_t collected = 0;
    auto fetch = [&](size_t chunk) {
      for (size_t i = 0; i < chunk; ++i)
        self->receive(
          [&](ids& sub_result) {
            ++collected;
            result |= sub_result;
          },
          after(0s) >> [&] {
            FAIL("missing sub result #" << (i + 1) << " in partition #"
                                        << (collected + 1));
          }
        );
    };
    fetch(scheduled);
    while (collected < hits) {
      auto chunk = std::min(hits - collected, taste_count);
      self->send(index, query_id, chunk);
      run_exhaustively();
      fetch(chunk);
    }
    return result;
  }

  // Handle to the INDEX actor.
  caf::actor index;
};

} // namespace <anonymous>

FIXTURE_SCOPE(index_tests, fixture)

TEST(ingestion) {
  MESSAGE("ingest 1000 integers");
  auto src = detail::spawn_generator_source(sys, index, 1000, int_generator());
  run_exhaustively();
  MESSAGE("verify partition index");
  REQUIRE_EQUAL(state().part_index.size(), 10u);
  auto intervals = partition_intervals();
  CHECK_EQUAL(intervals[0], interval(epoch, epoch + 99s));
  CHECK_EQUAL(intervals[1], interval(epoch + 100s, epoch + 199s));
  CHECK_EQUAL(intervals[2], interval(epoch + 200s, epoch + 299s));
  CHECK_EQUAL(intervals[3], interval(epoch + 300s, epoch + 399s));
  CHECK_EQUAL(intervals[4], interval(epoch + 400s, epoch + 499s));
  CHECK_EQUAL(intervals[5], interval(epoch + 500s, epoch + 599s));
  CHECK_EQUAL(intervals[6], interval(epoch + 600s, epoch + 699s));
  CHECK_EQUAL(intervals[7], interval(epoch + 700s, epoch + 799s));
  CHECK_EQUAL(intervals[8], interval(epoch + 800s, epoch + 899s));
  CHECK_EQUAL(intervals[9], interval(epoch + 900s, epoch + 999s));
}

TEST(one-shot integer query result) {
  MESSAGE("fill first " << taste_count << " partitions");
  auto src = detail::spawn_generator_source(sys, index,
                                            partition_size * taste_count,
                                            int_generator(2));
  run_exhaustively();
  MESSAGE("query half of the values");
  self->send(index, unbox(to<expression>(":int == 1")));
  run_exhaustively();
  auto [query_id, hits, scheduled] = receive_query_id();
  CHECK_EQUAL(query_id, uuid::nil());
  CHECK_EQUAL(hits, taste_count);
  CHECK_EQUAL(scheduled, taste_count);
  ids expected_result;
  for (size_t i = 0; i < (partition_size * taste_count) / 2; ++i) {
    expected_result.append_bit(false);
    expected_result.append_bit(true);
  }
  auto result = receive_result(query_id, hits, scheduled);
  CHECK_EQUAL(result, expected_result);
}

TEST(iterable integer query result) {
  MESSAGE("fill first " << (taste_count * 3) << " partitions");
  auto src = detail::spawn_generator_source(sys, index,
                                            partition_size * taste_count * 3,
                                            int_generator(2));
  run_exhaustively();
  MESSAGE("query half of the values");
  self->send(index, unbox(to<expression>(":int == 1")));
  run_exhaustively();
  auto [query_id, hits, scheduled] = receive_query_id();
  CHECK_NOT_EQUAL(query_id, uuid::nil());
  CHECK_EQUAL(hits, taste_count * 3);
  CHECK_EQUAL(scheduled, taste_count);
  ids expected_result;
  for (size_t i = 0; i < (partition_size * taste_count * 3) / 2; ++i) {
    expected_result.append_bit(false);
    expected_result.append_bit(true);
  }
  MESSAGE("collect results");
  auto result = receive_result(query_id, hits, scheduled);
  CHECK_EQUAL(result, expected_result);
}

FIXTURE_SCOPE_END()
