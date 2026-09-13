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

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/detail/component_composer.hpp"

using namespace omni_plan;
using namespace omni_plan_cache::detail;

namespace {

class MockAction : public pddl::Action {
public:
  MockAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params)
      : Action(name, params) {}
  pddl::ActionStatus run(const std::vector<std::string> &) override {
    return pddl::ActionStatus::SUCCEEDED;
  }
  void cancel() override {}
};

pddl::Domain make_domain() {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("done", {"?l"}));
  auto move = std::make_shared<MockAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}});
  move->add_effect(pddl::Type::END, "at", {"?r", "?to"});
  move->add_effect(pddl::Type::END, "at", {"?r", "?from"}, true);
  domain.add_action(move);
  return domain;
}

} // namespace

TEST(ComponentComposerTest, ParallelIndependentDetection) {
  EXPECT_TRUE(ComponentComposer::can_solve_in_parallel(
      {std::set<std::string>{"a"}, std::set<std::string>{"b"},
       std::set<std::string>{"c"}}));
  EXPECT_FALSE(ComponentComposer::can_solve_in_parallel(
      {std::set<std::string>{"robot1", "a"},
       std::set<std::string>{"robot1", "b"}}));
}

TEST(ComponentComposerTest, ComposesDisjointComponents) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  for (const auto *name : {"a", "b", "c"}) {
    problem.add_object(pddl::Object(name, "location"));
    problem.add_goal(pddl::Predicate("done", {name}));
  }

  int solver_calls = 0;
  ComponentComposer composer(
      ComponentComposer::Options{},
      [&](const pddl::Domain &d, const pddl::Problem &) {
        ++solver_calls;
        pddl::Plan plan;
        plan.set_has_solution(true);
        plan.add_action(d.get_actions().at("move"), {"robot1", "a", "b"}, 0.0f);
        return plan;
      });

  pddl::Plan out;
  const std::set<pddl::Predicate> relevant;
  const std::set<std::string> static_predicates;
  EXPECT_TRUE(composer.compose(domain, problem, relevant, static_predicates, out));
  EXPECT_EQ(solver_calls, 3);
  EXPECT_TRUE(out.has_solution());
  EXPECT_EQ(out.size(), 3u);
}

TEST(ComponentComposerTest, SequentialCompositionSimulatesEarlierEffects) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("a", "location"));
  problem.add_object(pddl::Object("l1", "location"));
  problem.add_goal(pddl::Predicate("done", {"a"}));
  problem.add_goal(pddl::Predicate("at", {"robot1", "l1"}));

  std::vector<std::set<pddl::Predicate>> observed_facts;
  ComponentComposer composer(
      ComponentComposer::Options{},
      [&](const pddl::Domain &d, const pddl::Problem &p) {
        const size_t call = observed_facts.size();
        observed_facts.push_back(p.get_facts());
        pddl::Plan plan;
        plan.set_has_solution(true);
        if (call == 0) {
          plan.add_action(d.get_actions().at("move"), {"robot1", "a", "l1"},
                          0.0f);
        }
        return plan;
      });

  pddl::Plan out;
  EXPECT_TRUE(composer.compose(domain, problem, {}, {}, out));
  EXPECT_TRUE(out.has_solution());

  const pddl::Predicate reached("at", {"robot1", "l1"});
  ASSERT_GE(observed_facts.size(), 2u);
  bool carried_forward = false;
  for (size_t i = 1; i < observed_facts.size(); ++i) {
    carried_forward = carried_forward || observed_facts[i].count(reached) > 0;
  }
  EXPECT_TRUE(carried_forward);
}

TEST(ComponentComposerTest, RejectsTooManyGoalsPerComponent) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  for (int i = 0; i < 5; ++i) {
    const std::string loc = "l" + std::to_string(i);
    problem.add_object(pddl::Object(loc, "location"));
    problem.add_goal(pddl::Predicate("at", {"robot1", loc}));
  }
  // A second, disjoint component ensures the rejection comes from the
  // per-component goal limit rather than the single-component guard.
  problem.add_object(pddl::Object("lone", "location"));
  problem.add_goal(pddl::Predicate("done", {"lone"}));

  ComponentComposer composer(ComponentComposer::Options{},
                             [](const pddl::Domain &, const pddl::Problem &) {
                               pddl::Plan plan;
                               plan.set_has_solution(true);
                               return plan;
                             });
  pddl::Plan out;
  EXPECT_FALSE(composer.compose(domain, problem, {}, {}, out));
}
