// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/detail/structural_keyer.hpp"

using namespace omni_plan;

namespace {

using Keyer = omni_plan_cache::detail::StructuralKeyer;

/// @brief Minimal executable action used only to build PDDL domains.
class DummyAction : public pddl::Action {
public:
  DummyAction(const std::string &name, float duration,
              std::vector<std::pair<std::string, std::string>> params)
      : Action(name, duration, params) {}

  pddl::ActionStatus run(const std::vector<std::string> &) override {
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override {}
};

/// @brief Nav domain whose move action has the given duration.
pddl::Domain make_duration_domain(float duration) {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("robot");
  domain.add_type("location");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_action(std::make_shared<DummyAction>(
      "move", duration,
      std::vector<std::pair<std::string, std::string>>{
          {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}}));
  return domain;
}

pddl::Problem make_two_location_problem() {
  pddl::Problem problem;
  problem.add_object(pddl::Object("r", "robot"));
  problem.add_object(pddl::Object("l1", "location"));
  problem.add_object(pddl::Object("l2", "location"));
  problem.add_fact(pddl::Predicate("at", {"r", "l1"}));
  problem.add_goal(pddl::Predicate("at", {"r", "l2"}));
  return problem;
}

} // namespace

TEST(StructuralKeyerTest, AbstractRoleKeys) {
  std::set<pddl::Object> objects = {
      pddl::Object("robot1", "robot"), pddl::Object("loc1", "location"),
      pddl::Object("loc2", "location"), pddl::Object("loc3", "location")};
  auto groups =
      omni_plan_cache::detail::StructuralKeyer::group_objects_by_type(objects);
  std::set<pddl::Predicate> facts = {
      pddl::Predicate("at", {"robot1", "loc1"}),
      pddl::Predicate("connected", {"loc1", "loc2"}),
      pddl::Predicate("connected", {"loc2", "loc3"})};
  std::set<pddl::Predicate> goals = {pddl::Predicate("at", {"robot1", "loc3"})};

  auto keys = omni_plan_cache::detail::StructuralKeyer::compute_role_keys(
      groups, facts, goals);
  EXPECT_EQ(keys["robot1"], "at_0_0|at_0_1|");
  EXPECT_EQ(keys["loc1"], "at_1_0|connected_0_0|");
  EXPECT_EQ(keys["loc2"], "connected_0_0|connected_1_0|");
  EXPECT_EQ(keys["loc3"], "at_1_1|connected_1_0|");
}

TEST(StructuralKeyerTest, PrepareFiltersAndSorts) {
  std::set<pddl::Object> objects = {
      pddl::Object("robot1", "robot"), pddl::Object("loc1", "location"),
      pddl::Object("loc2", "location"), pddl::Object("unused", "location")};
  std::set<pddl::Predicate> facts = {
      pddl::Predicate("at", {"robot1", "loc1"}),
      pddl::Predicate("connected", {"loc1", "loc2"})};
  std::set<pddl::Predicate> goals = {pddl::Predicate("at", {"robot1", "loc2"})};

  auto prepared = omni_plan_cache::detail::StructuralKeyer::prepare(
      objects, facts, goals, false);

  ASSERT_EQ(prepared.objects_by_type.size(), 2u);
  for (const auto &group : prepared.objects_by_type) {
    if (group.type == "location") {
      ASSERT_EQ(group.names.size(), 2u);
      EXPECT_EQ(group.names[0], "loc1");
      EXPECT_EQ(group.names[1], "loc2");
    }
  }
  EXPECT_EQ(prepared.placeholder_to_original.at("__obj_location_0__"), "loc1");
  EXPECT_EQ(prepared.placeholder_to_original.at("__obj_location_1__"), "loc2");
  EXPECT_EQ(prepared.name_to_alias.at("loc1"), "location_0");
  EXPECT_FALSE(prepared.role_keys.at("loc1").empty());
}

TEST(StructuralKeyerTest, ExactKeyDetectsDifferentFacts) {
  pddl::Domain domain;
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  pddl::Problem a;
  a.add_object(pddl::Object("r", "robot"));
  a.add_object(pddl::Object("l1", "location"));
  a.add_object(pddl::Object("l2", "location"));
  a.add_fact(pddl::Predicate("at", {"r", "l1"}));
  a.add_goal(pddl::Predicate("at", {"r", "l2"}));

  pddl::Problem b = a;
  b.add_fact(pddl::Predicate("at", {"r", "l2"}));

  EXPECT_EQ(omni_plan_cache::detail::StructuralKeyer::exact_key(domain, a),
            omni_plan_cache::detail::StructuralKeyer::exact_key(domain, a));
  EXPECT_NE(omni_plan_cache::detail::StructuralKeyer::exact_key(domain, a),
            omni_plan_cache::detail::StructuralKeyer::exact_key(domain, b));
}

// Regression: a negated predicate must produce different role keys and a
// different structural key than its positive counterpart.
TEST(StructuralKeyerTest, NegatedPredicateChangesKeys) {
  pddl::Domain domain;
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("connected", {"?l1", "?l2"}));

  auto make_problem = [](bool negated_goal) {
    pddl::Problem problem;
    problem.add_object(pddl::Object("r", "robot"));
    problem.add_object(pddl::Object("l1", "location"));
    problem.add_object(pddl::Object("l2", "location"));
    problem.add_fact(pddl::Predicate("at", {"r", "l1"}));
    problem.add_fact(pddl::Predicate("connected", {"l1", "l2"}));
    problem.add_goal(pddl::Predicate("at", {"r", "l2"}, negated_goal));
    return problem;
  };

  const pddl::Problem positive = make_problem(false);
  const pddl::Problem negative = make_problem(true);

  const auto keys_positive = Keyer::compute_role_keys(
      Keyer::group_objects_by_type(positive.get_objects()),
      positive.get_facts(), positive.get_goals());
  const auto keys_negative = Keyer::compute_role_keys(
      Keyer::group_objects_by_type(negative.get_objects()),
      negative.get_facts(), negative.get_goals());
  EXPECT_NE(keys_positive, keys_negative);

  const auto prepared_positive =
      Keyer::prepare(positive.get_objects(), positive.get_facts(),
                     positive.get_goals(), false);
  const auto prepared_negative =
      Keyer::prepare(negative.get_objects(), negative.get_facts(),
                     negative.get_goals(), false);
  EXPECT_NE(
      Keyer::compute_key(domain, positive, prepared_positive.objects_by_type,
                         prepared_positive.role_keys, nullptr),
      Keyer::compute_key(domain, negative, prepared_negative.objects_by_type,
                         prepared_negative.role_keys, nullptr));
}

// Regression: durations that differ by less than 1e-6 (one float ULP at 1.0)
// must produce different exact keys; std::to_string(float) collapses them.
TEST(StructuralKeyerTest, ExactKeyDistinguishesSubMicrosecondDurations) {
  const float base = 1.0f;
  const float next = std::nextafter(base, 2.0f);
  ASSERT_NE(base, next);
  // The serialized duration must be identical for both floats, otherwise the
  // regression would not reproduce the bug.
  ASSERT_EQ(std::to_string(base), std::to_string(next));

  const pddl::Problem problem = make_two_location_problem();
  EXPECT_NE(Keyer::exact_key(make_duration_domain(base), problem),
            Keyer::exact_key(make_duration_domain(next), problem));
}

// Regression: the public string-PDDL overload and the runtime Domain overload
// must produce the same structural key.
TEST(StructuralKeyerTest, DomainAndStringOverloadsAgree) {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("robot");
  domain.add_type("location");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("connected", {"?l1", "?l2"}));
  domain.add_action(std::make_shared<DummyAction>(
      "move", 10.0f,
      std::vector<std::pair<std::string, std::string>>{
          {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}}));

  pddl::Problem problem = make_two_location_problem();
  problem.add_fact(pddl::Predicate("connected", {"l1", "l2"}));

  const auto prepared = Keyer::prepare(
      problem.get_objects(), problem.get_facts(), problem.get_goals(), false);
  EXPECT_EQ(Keyer::compute_key(domain, problem, prepared.objects_by_type,
                               prepared.role_keys, nullptr),
            Keyer::compute_key(domain.to_pddl(), problem,
                               prepared.objects_by_type, prepared.role_keys,
                               nullptr));
}
