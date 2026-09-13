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

#include <set>
#include <string>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/detail/structural_keyer.hpp"

using namespace omni_plan;

TEST(StructuralKeyerTest, AbstractRoleKeys) {
  std::set<pddl::Object> objects = {
      pddl::Object("robot1", "robot"), pddl::Object("loc1", "location"),
      pddl::Object("loc2", "location"), pddl::Object("loc3", "location")};
  auto groups = omni_plan_cache::detail::StructuralKeyer::group_objects_by_type(
      objects);
  std::set<pddl::Predicate> facts = {
      pddl::Predicate("at", {"robot1", "loc1"}),
      pddl::Predicate("connected", {"loc1", "loc2"}),
      pddl::Predicate("connected", {"loc2", "loc3"})};
  std::set<pddl::Predicate> goals = {
      pddl::Predicate("at", {"robot1", "loc3"})};

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
  std::set<pddl::Predicate> goals = {
      pddl::Predicate("at", {"robot1", "loc2"})};

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
