// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/detail/relevance_analyzer.hpp"

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
  domain.add_type("item");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("connected", {"?l1", "?l2"}));
  domain.add_predicate(pddl::Predicate("item_at", {"?i", "?l"}));

  auto move = std::make_shared<MockAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}});
  move->add_condition(pddl::Type::START, "at", {"?r", "?from"});
  move->add_condition(pddl::Type::START, "connected", {"?from", "?to"});
  move->add_effect(pddl::Type::END, "at", {"?r", "?to"});
  move->add_effect(pddl::Type::END, "at", {"?r", "?from"}, true);
  domain.add_action(move);
  return domain;
}

} // namespace

TEST(RelevanceAnalyzerTest, FiltersIrrelevantPredicatesAndFacts) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("loc1", "location"));
  problem.add_object(pddl::Object("loc2", "location"));
  problem.add_object(pddl::Object("item1", "item"));
  problem.add_fact(pddl::Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(pddl::Predicate("connected", {"loc1", "loc2"}));
  problem.add_fact(pddl::Predicate("item_at", {"item1", "loc1"}));
  problem.add_goal(pddl::Predicate("at", {"robot1", "loc2"}));

  const auto result = RelevanceAnalyzer::analyze(domain, problem, "robot");

  EXPECT_TRUE(result.relevant_predicates.count("at") == 1);
  EXPECT_TRUE(result.relevant_predicates.count("connected") == 1);
  EXPECT_TRUE(result.relevant_predicates.count("item_at") == 0);

  EXPECT_TRUE(result.full_static_predicates.count("connected") == 1);
  EXPECT_TRUE(result.full_static_predicates.count("item_at") == 1);
  EXPECT_TRUE(result.full_static_predicates.count("at") == 0);

  EXPECT_TRUE(result.static_predicates.count("connected") == 1);
  EXPECT_TRUE(result.static_predicates.count("at") == 0);

  EXPECT_TRUE(result.relevant_objects.count("robot1") == 1);
  EXPECT_TRUE(result.relevant_objects.count("loc1") == 1);
  EXPECT_TRUE(result.relevant_objects.count("loc2") == 1);

  EXPECT_TRUE(result.relevant_facts.count(
                  pddl::Predicate("at", {"robot1", "loc1"})) == 1);
  EXPECT_TRUE(result.relevant_facts.count(
                  pddl::Predicate("connected", {"loc1", "loc2"})) == 1);
  EXPECT_TRUE(result.relevant_facts.count(
                  pddl::Predicate("item_at", {"item1", "loc1"})) == 0);
}

TEST(RelevanceAnalyzerTest, RobotTypeControlsRobotInclusion) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("robot2", "robot"));
  problem.add_object(pddl::Object("loc1", "location"));
  problem.add_object(pddl::Object("loc2", "location"));
  problem.add_fact(pddl::Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(pddl::Predicate("connected", {"loc1", "loc2"}));
  problem.add_goal(pddl::Predicate("at", {"robot1", "loc2"}));

  const auto with_robot = RelevanceAnalyzer::analyze(domain, problem, "robot");
  const auto without_robot =
      RelevanceAnalyzer::analyze(domain, problem, "nonexistent");
  EXPECT_EQ(with_robot.relevant_objects.count("robot2"), 1u);
  EXPECT_EQ(without_robot.relevant_objects.count("robot2"), 0u);
}
