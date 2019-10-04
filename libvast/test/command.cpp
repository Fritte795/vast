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

#define SUITE command

#include "vast/test/test.hpp"

#include "vast/command.hpp"

#include <caf/actor_system_config.hpp>
#include <caf/make_message.hpp>
#include <caf/message.hpp>

#include "vast/system/version_command.hpp"

using namespace vast;

using namespace std::string_literals;

namespace {

caf::message foo(const command::invocation& invocation, caf::actor_system&) {
  CHECK_EQUAL(invocation.name(), "foo");
  return caf::make_message("foo");
}

caf::message bar(const command::invocation& invocation, caf::actor_system&) {
  CHECK_EQUAL(invocation.name(), "bar");
  return caf::make_message("bar");
}

struct fixture {
  command root;
  caf::actor_system_config cfg;
  caf::actor_system sys{cfg};
  command::invocation invocation;
  caf::settings& options;

  fixture() : options(invocation.options) {
    root.name = "vast";
  }

  caf::variant<caf::none_t, std::string, caf::error>
  exec(std::string str, const command::factory& factory) {
    options.clear();
    std::vector<std::string> xs;
    caf::split(xs, str, ' ', caf::token_compress_on);
    auto expected_inv = parse(root, xs.begin(), xs.end());
    if (!expected_inv)
      return expected_inv.error();
    invocation = std::move(*expected_inv);
    auto result = run(invocation, sys, factory);
    if (!result)
      return result.error();
    if (result->empty())
      return caf::none;
    if (result->match_elements<std::string>())
      return result->get_as<std::string>(0);
    if (result->match_elements<caf::error>())
      return result->get_as<caf::error>(0);
    FAIL("command returned an unexpected result");
  }
};

template <class T>
bool is_error(const T& x) {
  return caf::holds_alternative<caf::error>(x);
}

} // namespace <anonymous>

FIXTURE_SCOPE(command_tests, fixture)

TEST(names) {
  using svec = std::vector<std::string>;
  auto aa = root.add("a")->add("aa");
  aa->add("aaa");
  aa->add("aab");
  CHECK_EQUAL(aa->name, "aa");
  root.add("b");
  svec names;
  for_each(root, [&](auto& cmd) { names.emplace_back(cmd.full_name()); });
  CHECK_EQUAL(names, svec({"vast", "a", "a aa", "a aa aaa", "a aa aab", "b"}));
}

TEST(flat command invocation) {
  command::factory factory{};
  auto fptr = root
                .add("foo", command::opts()
                              .add<int>("value,v", "some int")
                              .add<bool>("flag", "some flag"))
                ->run(foo, factory);
  CHECK_EQUAL(fptr->name, "foo");
  CHECK_EQUAL(fptr->full_name(), "foo");
  auto bptr = root.add("bar")->run(bar, factory);
  CHECK_EQUAL(bptr->name, "bar");
  CHECK_EQUAL(bptr->full_name(), "bar");
  CHECK(is_error(exec("nop", factory)));
  CHECK(is_error(exec("bar --flag -v 42", factory)));
  CHECK(is_error(exec("--flag bar", factory)));
  CHECK_EQUAL(get_or(options, "flag", false), false);
  CHECK_EQUAL(get_or(options, "value", 0), 0);
  CHECK_EQUAL(exec("bar", factory), "bar"s);
  CHECK_EQUAL(exec("foo --flag -v 42", factory), "foo"s);
  CHECK_EQUAL(get_or(options, "flag", false), true);
  CHECK_EQUAL(get_or(options, "value", 0), 42);
}

TEST(nested command invocation) {
  command::factory factory{};
  auto fptr = root
                .add("foo", command::opts()
                              .add<int>("value,v", "some int")
                              .add<bool>("flag", "some flag"))
                ->run(foo, factory);
  CHECK_EQUAL(fptr->name, "foo");
  CHECK_EQUAL(fptr->full_name(), "foo");
  auto bptr = fptr->add("bar")->run(bar, factory);
  CHECK_EQUAL(bptr->name, "bar");
  CHECK_EQUAL(bptr->full_name(), "foo bar");
  CHECK(is_error(exec("nop", factory)));
  CHECK(is_error(exec("bar --flag -v 42", factory)));
  CHECK(is_error(exec("foo --flag -v 42 --other-flag", factory)));
  CHECK_EQUAL(exec("foo --flag -v 42", factory), "foo"s);
  CHECK_EQUAL(get_or(options, "flag", false), true);
  CHECK_EQUAL(get_or(options, "value", 0), 42);
  CHECK_EQUAL(exec("foo --flag -v 42 bar", factory), "bar"s);
  CHECK_EQUAL(get_or(options, "flag", false), true);
  CHECK_EQUAL(get_or(options, "value", 0), 42);
  // Setting the command function to nullptr prohibits calling it directly.
  fptr->run(nullptr, factory);
  CHECK(is_error(exec("foo --flag -v 42", factory)));
  // Subcommands of course still work.
  CHECK_EQUAL(exec("foo --flag -v 42 bar", factory), "bar"s);
}

TEST(version command) {
  command::factory factory{};
  root.add("version", command::opts())->run(system::version_command, factory);
  CHECK_EQUAL(exec("version", factory), caf::none);
}

FIXTURE_SCOPE_END()
