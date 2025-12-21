// Copyright (C) 2025 Miguel Ángel González Santamarta
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
#include <string>
#include <vector>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/plan.hpp"
#include "easy_plan_vhpop/vhpop_planner.hpp"

using namespace easy_plan_vhpop;

// Mock Action class for testing
class MockAction : public easy_plan::pddl::Action {
public:
  MockAction(
      const std::string &name,
      const std::vector<std::pair<std::string, std::string>> &params = {})
      : Action(name, params), cancel_called_(false) {}

  easy_plan::pddl::ActionStatus
  run(std::vector<std::string> /*params*/) override {
    return easy_plan::pddl::ActionStatus::SUCCEED;
  }

  void cancel() override { cancel_called_ = true; }

  bool cancel_called_;
};

class VhpopPlannerTest : public ::testing::Test {
protected:
  void SetUp() override { planner_ = std::make_unique<VhpopPlanner>(); }

  std::unique_ptr<VhpopPlanner> planner_;

  // Simple PDDL domain for testing
  const std::string simple_domain_ = R"(
(define (domain simple-domain)
  (:requirements :strips :typing)
  (:types location robot)
  (:predicates
    (at ?r - robot ?l - location)
    (connected ?l1 - location ?l2 - location)
  )
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and (not (at ?r ?from)) (at ?r ?to))
  )
)
)";

  // Simple PDDL problem for testing
  const std::string simple_problem_ = R"(
(define (problem simple-problem)
  (:domain simple-domain)
  (:objects
    robot1 - robot
    loc1 loc2 - location
  )
  (:init
    (at robot1 loc1)
    (connected loc1 loc2)
  )
  (:goal (at robot1 loc2))
)
)";

  // Invalid domain for testing failure cases
  const std::string invalid_domain_ = R"(
(define (domain invalid)
  (:requirements :strips :typing)
  (:predicates (invalid-pred
)
)";

  // Unsolvable problem for testing
  const std::string unsolvable_problem_ = R"(
(define (problem unsolvable-problem)
  (:domain simple-domain)
  (:objects
    robot1 - robot
    loc1 loc2 - location
  )
  (:init
    (at robot1 loc1)
  )
  (:goal (at robot1 loc2))
)
)";

  std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>>
  create_actions() {
    std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>> actions;
    actions["move"] = std::make_shared<MockAction>("move");
    return actions;
  }
};

// Test: VhpopPlanner constructor
TEST_F(VhpopPlannerTest, ConstructorCreatesPlanner) {
  EXPECT_NE(planner_, nullptr);
}

// Test: get_plan with invalid domain returns failed plan
TEST_F(VhpopPlannerTest, GetPlanWithInvalidDomainReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->get_plan(invalid_domain_, simple_problem_, actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: get_plan with empty domain returns failed plan
TEST_F(VhpopPlannerTest, GetPlanWithEmptyDomainReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->get_plan("", "", actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: get_plan with empty actions map handles gracefully
TEST_F(VhpopPlannerTest, GetPlanWithEmptyActionsMap) {
  std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>> empty_actions;
  auto plan =
      planner_->get_plan(simple_domain_, simple_problem_, empty_actions);

  // Plan might succeed at VHPOP level but actions won't be mapped
  // The behavior depends on VHPOP availability
}

// Test: get_plan with unsolvable problem returns failed plan
TEST_F(VhpopPlannerTest, GetPlanWithUnsolvableProblemReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->get_plan(simple_domain_, unsolvable_problem_, actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: Plan size is 0 for failed plans
TEST_F(VhpopPlannerTest, FailedPlanHasSizeZero) {
  auto actions = create_actions();
  auto plan = planner_->get_plan(invalid_domain_, simple_problem_, actions);

  EXPECT_EQ(plan.size(), 0u);
}

// Test: Multiple calls to get_plan work correctly
TEST_F(VhpopPlannerTest, MultiplePlannerCalls) {
  auto actions = create_actions();

  auto plan1 = planner_->get_plan(invalid_domain_, simple_problem_, actions);
  auto plan2 = planner_->get_plan(invalid_domain_, simple_problem_, actions);

  EXPECT_FALSE(plan1.has_solution());
  EXPECT_FALSE(plan2.has_solution());
}

// Integration test: Valid domain and problem (requires VHPOP to be installed)
TEST_F(VhpopPlannerTest, ValidDomainAndProblemReturnsPlan) {
  auto actions = create_actions();
  auto plan = planner_->get_plan(simple_domain_, simple_problem_, actions);

  if (plan.has_solution()) {
    EXPECT_GT(plan.size(), 0u);

    // Verify the plan contains the expected action
    auto action = plan.get_action(0);
    EXPECT_EQ(action->get_name(), "move");

    auto params = plan.get_action_params(0);
    EXPECT_EQ(params.size(), 3u);
  }
}

// Integration test: Verify plan actions are correctly mapped
TEST_F(VhpopPlannerTest, PlanActionsCorrectlyMapped) {
  auto actions = create_actions();
  auto plan = planner_->get_plan(simple_domain_, simple_problem_, actions);

  if (plan.has_solution()) {
    for (size_t i = 0; i < plan.size(); ++i) {
      auto [action, params] = plan.get_action_with_params(i);
      EXPECT_NE(action, nullptr);
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
