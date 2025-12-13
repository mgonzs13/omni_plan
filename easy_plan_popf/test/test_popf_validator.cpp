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
#include "easy_plan/plan.hpp"
#include "easy_plan_popf/popf_validator.hpp"

using namespace easy_plan_popf;

// Mock Action class for testing
class MockAction : public easy_plan::pddl::Action {
public:
  MockAction(const std::string &name,
             const std::vector<easy_plan::pddl::Parameter> &params = {})
      : Action(name, params), cancel_called_(false) {}

  easy_plan::pddl::ActionStatus
  run(std::vector<std::string> /*params*/) override {
    return easy_plan::pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { cancel_called_ = true; }

  bool cancel_called_;
};

class PopfValidatorTest : public ::testing::Test {
protected:
  void SetUp() override { validator_ = std::make_unique<PopfValidator>(); }

  std::unique_ptr<PopfValidator> validator_;

  // Simple PDDL domain for testing
  const std::string simple_domain_ = R"(
(define (domain simple-domain)
  (:requirements :strips)
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
  (:requirements :strips)
  (:predicates (invalid-pred
)
)";

  easy_plan::Plan create_valid_plan() {
    easy_plan::Plan plan(true);
    auto move_action = std::make_shared<MockAction>("move");
    plan.add_action(move_action, {"robot1", "loc1", "loc2"});
    return plan;
  }

  easy_plan::Plan create_invalid_plan() {
    easy_plan::Plan plan(true);
    auto move_action = std::make_shared<MockAction>("move");
    // Invalid: robot not at loc2 initially
    plan.add_action(move_action, {"robot1", "loc2", "loc1"});
    return plan;
  }

  easy_plan::Plan create_empty_plan() { return easy_plan::Plan(true); }

  easy_plan::Plan create_multi_action_plan() {
    easy_plan::Plan plan(true);
    auto move_action = std::make_shared<MockAction>("move");
    plan.add_action(move_action, {"robot1", "loc1", "loc2"});
    plan.add_action(move_action, {"robot1", "loc2", "loc1"});
    return plan;
  }
};

// Test: PopfValidator constructor
TEST_F(PopfValidatorTest, ConstructorCreatesValidator) {
  EXPECT_NE(validator_, nullptr);
}

// Test: plan_to_string with empty plan
TEST_F(PopfValidatorTest, PlanToStringEmptyPlan) {
  auto plan = create_empty_plan();
  std::string plan_str = validator_->plan_to_string(plan);

  EXPECT_EQ(plan_str, "");
}

// Test: plan_to_string with single action
TEST_F(PopfValidatorTest, PlanToStringSingleAction) {
  auto plan = create_valid_plan();
  std::string plan_str = validator_->plan_to_string(plan);

  EXPECT_TRUE(plan_str.find("1: (move robot1 loc1 loc2)") != std::string::npos);
}

// Test: plan_to_string with multiple actions
TEST_F(PopfValidatorTest, PlanToStringMultipleActions) {
  auto plan = create_multi_action_plan();
  std::string plan_str = validator_->plan_to_string(plan);

  EXPECT_TRUE(plan_str.find("1: (move robot1 loc1 loc2)") != std::string::npos);
  EXPECT_TRUE(plan_str.find("2: (move robot1 loc2 loc1)") != std::string::npos);
}

// Test: plan_to_string format is correct
TEST_F(PopfValidatorTest, PlanToStringFormat) {
  auto plan = create_valid_plan();
  std::string plan_str = validator_->plan_to_string(plan);

  // Should have format: "index: (action_name params...)\n"
  EXPECT_TRUE(plan_str.find(": (") != std::string::npos);
  EXPECT_TRUE(plan_str.find(")\n") != std::string::npos);
}

// Test: plan_to_string preserves action order
TEST_F(PopfValidatorTest, PlanToStringPreservesOrder) {
  auto plan = create_multi_action_plan();
  std::string plan_str = validator_->plan_to_string(plan);

  size_t pos1 = plan_str.find("1: (move");
  size_t pos2 = plan_str.find("2: (move");

  EXPECT_LT(pos1, pos2);
}

// Test: validate_plan with empty domain returns false
TEST_F(PopfValidatorTest, ValidatePlanEmptyDomainReturnsFalse) {
  auto plan = create_valid_plan();
  bool result = validator_->validate_plan("", simple_problem_, plan);

  EXPECT_FALSE(result);
}

// Test: validate_plan with empty problem returns false
TEST_F(PopfValidatorTest, ValidatePlanEmptyProblemReturnsFalse) {
  auto plan = create_valid_plan();
  bool result = validator_->validate_plan(simple_domain_, "", plan);

  EXPECT_FALSE(result);
}

// Test: validate_plan with invalid domain returns false
TEST_F(PopfValidatorTest, ValidatePlanInvalidDomainReturnsFalse) {
  auto plan = create_valid_plan();
  bool result =
      validator_->validate_plan(invalid_domain_, simple_problem_, plan);

  EXPECT_FALSE(result);
}

// Test: validate_plan with empty plan
TEST_F(PopfValidatorTest, ValidatePlanEmptyPlan) {
  auto plan = create_empty_plan();
  bool result =
      validator_->validate_plan(simple_domain_, simple_problem_, plan);

  // Empty plan might be valid or invalid depending on validator behavior
  // and whether the goal is already satisfied
}

// Test: Multiple calls to validate_plan work correctly
TEST_F(PopfValidatorTest, MultipleValidatorCalls) {
  auto plan = create_valid_plan();

  bool result1 =
      validator_->validate_plan(invalid_domain_, simple_problem_, plan);
  bool result2 =
      validator_->validate_plan(invalid_domain_, simple_problem_, plan);

  EXPECT_FALSE(result1);
  EXPECT_FALSE(result2);
}

// Test: Plan without solution
TEST_F(PopfValidatorTest, PlanWithoutSolution) {
  easy_plan::Plan plan(false);
  std::string plan_str = validator_->plan_to_string(plan);

  EXPECT_EQ(plan_str, "");
}

// Test: Action with no parameters
TEST_F(PopfValidatorTest, ActionWithNoParameters) {
  easy_plan::Plan plan(true);
  auto action = std::make_shared<MockAction>("no_param_action");
  plan.add_action(action);

  std::string plan_str = validator_->plan_to_string(plan);
  EXPECT_TRUE(plan_str.find("1: (no_param_action)") != std::string::npos);
}

// Integration test: Valid plan validation (requires POPF validate to be
// installed) This test is marked as DISABLED because it requires external
// dependency
TEST_F(PopfValidatorTest, DISABLED_ValidPlanPassesValidation) {
  auto plan = create_valid_plan();
  bool result =
      validator_->validate_plan(simple_domain_, simple_problem_, plan);

  EXPECT_TRUE(result);
}

// Integration test: Invalid plan fails validation (requires POPF validate to be
// installed)
TEST_F(PopfValidatorTest, DISABLED_InvalidPlanFailsValidation) {
  auto plan = create_invalid_plan();
  bool result =
      validator_->validate_plan(simple_domain_, simple_problem_, plan);

  EXPECT_FALSE(result);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
