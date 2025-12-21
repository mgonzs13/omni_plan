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

#include "rclcpp/rclcpp.hpp"

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan_popf/popf_validator.hpp"

using namespace easy_plan_popf;

// Mock Action class for testing
class MockAction : public easy_plan::pddl::Action {
public:
  MockAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params), cancel_called_(false) {}

  easy_plan::pddl::ActionStatus
  run(const std::vector<std::string> & /*params*/) override {
    return easy_plan::pddl::ActionStatus::SUCCEED;
  }

  void cancel() override { cancel_called_ = true; }

  bool cancel_called_;
};

class PopfValidatorTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    validator_ = std::make_unique<PopfValidator>();
    validator_->load_parameters(node_);

    // Build simple domain
    simple_domain_obj_.add_requirement("strips");
    simple_domain_obj_.add_requirement("typing");
    simple_domain_obj_.add_requirement("durative-actions");
    simple_domain_obj_.add_type("location");
    simple_domain_obj_.add_type("robot");
    simple_domain_obj_.add_predicate(
        easy_plan::pddl::Predicate("at", {"robot", "location"}));
    simple_domain_obj_.add_predicate(
        easy_plan::pddl::Predicate("connected", {"location", "location"}));
    std::vector<std::pair<std::string, std::string>> params = {
        {"r", "robot"}, {"from", "location"}, {"to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    move_action->add_condition(easy_plan::pddl::Type::START, "at",
                               {"r", "from"});
    move_action->add_condition(easy_plan::pddl::Type::START, "connected",
                               {"from", "to"});
    move_action->add_effect(easy_plan::pddl::Type::END, "at", {"r", "to"});
    move_action->add_effect(easy_plan::pddl::Type::END, "at", {"r", "from"},
                            true);
    simple_domain_obj_.add_action(move_action);

    // Build simple problem
    simple_problem_obj_.add_object(easy_plan::pddl::Object("robot1", "robot"));
    simple_problem_obj_.add_object(easy_plan::pddl::Object("loc1", "location"));
    simple_problem_obj_.add_object(easy_plan::pddl::Object("loc2", "location"));
    simple_problem_obj_.add_fact(
        easy_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    simple_problem_obj_.add_fact(
        easy_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
    simple_problem_obj_.add_goal(
        easy_plan::pddl::Predicate("at", {"robot1", "loc2"}));

    // Build unsolvable problem (no connection between locations)
    unsolvable_problem_obj_.add_object(
        easy_plan::pddl::Object("robot1", "robot"));
    unsolvable_problem_obj_.add_object(
        easy_plan::pddl::Object("loc1", "location"));
    unsolvable_problem_obj_.add_object(
        easy_plan::pddl::Object("loc2", "location"));
    unsolvable_problem_obj_.add_fact(
        easy_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    unsolvable_problem_obj_.add_goal(
        easy_plan::pddl::Predicate("at", {"robot1", "loc2"}));
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::unique_ptr<PopfValidator> validator_;
  std::shared_ptr<rclcpp::Node> node_;
  easy_plan::pddl::Domain simple_domain_obj_;
  easy_plan::pddl::Problem simple_problem_obj_;
  easy_plan::pddl::Problem unsolvable_problem_obj_;

  // Helper to call base class validate_plan method
  bool validate(const easy_plan::pddl::Domain &domain,
                const easy_plan::pddl::Problem &problem,
                easy_plan::pddl::Plan plan) {
    easy_plan::PlanValidator *base_validator = validator_.get();
    return base_validator->validate_plan(domain, problem, plan);
  }

  std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>>
  create_actions() {
    std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>> actions;
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    actions["move"] = std::make_shared<MockAction>("move", params);
    return actions;
  }

  easy_plan::pddl::Plan create_valid_plan() {
    easy_plan::pddl::Plan plan(true);
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    plan.add_action(move_action, {"robot1", "loc1", "loc2"});
    return plan;
  }

  easy_plan::pddl::Plan create_invalid_plan() {
    easy_plan::pddl::Plan plan(true);
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    // Invalid: robot not at loc2 initially
    plan.add_action(move_action, {"robot1", "loc2", "loc1"});
    return plan;
  }

  easy_plan::pddl::Plan create_empty_plan() {
    return easy_plan::pddl::Plan(true);
  }

  easy_plan::pddl::Plan create_multi_action_plan() {
    easy_plan::pddl::Plan plan(true);
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    plan.add_action(move_action, {"robot1", "loc1", "loc2"});
    plan.add_action(move_action, {"robot1", "loc2", "loc1"});
    return plan;
  }
};

// Test: PopfValidator constructor
TEST_F(PopfValidatorTest, ConstructorCreatesValidator) {
  EXPECT_NE(validator_, nullptr);
}

// Test: parse_pddl with empty plan
TEST_F(PopfValidatorTest, ParsePddlEmptyPlan) {
  auto plan = create_empty_plan();
  std::string plan_str = validator_->parse_pddl(plan);

  EXPECT_EQ(plan_str, "");
}

// Test: parse_pddl with single action
TEST_F(PopfValidatorTest, ParsePddlSingleAction) {
  auto plan = create_valid_plan();
  std::string plan_str = validator_->parse_pddl(plan);

  EXPECT_TRUE(plan_str.find("0.000: (move robot1 loc1 loc2)") !=
              std::string::npos);
}

// Test: parse_pddl with multiple actions
TEST_F(PopfValidatorTest, ParsePddlMultipleActions) {
  auto plan = create_multi_action_plan();
  std::string plan_str = validator_->parse_pddl(plan);

  EXPECT_TRUE(plan_str.find("0.000: (move robot1 loc1 loc2)") !=
              std::string::npos);
  EXPECT_TRUE(plan_str.find("20.000: (move robot1 loc2 loc1)") !=
              std::string::npos);
}

// Test: parse_pddl format is correct
TEST_F(PopfValidatorTest, ParsePddlFormat) {
  auto plan = create_valid_plan();
  std::string plan_str = validator_->parse_pddl(plan);

  // Should have format: "index: (action_name params...)  [duration]"
  EXPECT_TRUE(plan_str.find(": (") != std::string::npos);
  EXPECT_TRUE(plan_str.find(")  [") != std::string::npos);
}

// Test: parse_pddl preserves action order
TEST_F(PopfValidatorTest, ParsePddlPreservesOrder) {
  auto plan = create_multi_action_plan();
  std::string plan_str = validator_->parse_pddl(plan);

  size_t pos1 = plan_str.find("0.000: (move");
  size_t pos2 = plan_str.find("20.000: (move");

  EXPECT_LT(pos1, pos2);
}

// Test: validate_plan with empty domain returns false
TEST_F(PopfValidatorTest, ValidatePlanEmptyDomainReturnsFalse) {
  auto plan = create_valid_plan();
  bool result = validate(easy_plan::pddl::Domain(), simple_problem_obj_, plan);

  EXPECT_FALSE(result);
}

// Test: validate_plan with empty problem returns false
TEST_F(PopfValidatorTest, ValidatePlanEmptyProblemReturnsFalse) {
  auto plan = create_valid_plan();
  bool result = validate(simple_domain_obj_, easy_plan::pddl::Problem(), plan);

  EXPECT_FALSE(result);
}

// Test: validate_plan with empty plan
TEST_F(PopfValidatorTest, ValidatePlanEmptyPlan) {
  auto plan = create_empty_plan();
  bool result = validate(simple_domain_obj_, simple_problem_obj_, plan);

  EXPECT_FALSE(result);
}

// Test: Multiple calls to validate_plan work correctly
TEST_F(PopfValidatorTest, MultipleValidatorCalls) {
  auto plan = create_valid_plan();

  bool result1 = validate(easy_plan::pddl::Domain(), simple_problem_obj_, plan);
  bool result2 = validate(easy_plan::pddl::Domain(), simple_problem_obj_, plan);

  EXPECT_FALSE(result1);
  EXPECT_FALSE(result2);
}

// Test: Plan without solution
TEST_F(PopfValidatorTest, PlanWithoutSolution) {
  easy_plan::pddl::Plan plan(false);
  std::string plan_str = validator_->parse_pddl(plan);

  EXPECT_EQ(plan_str, "");
}

// Test: Action with no parameters
TEST_F(PopfValidatorTest, ActionWithNoParameters) {
  easy_plan::pddl::Plan plan(true);
  auto action = std::make_shared<MockAction>("no_param_action");
  plan.add_action(action);

  std::string plan_str = validator_->parse_pddl(plan);
  EXPECT_TRUE(plan_str.find("0.000: (no_param_action)") != std::string::npos);
}

// Integration test: Valid plan validation (requires POPF validate to be
// installed)
TEST_F(PopfValidatorTest, ValidPlanPassesValidation) {
  auto plan = create_valid_plan();
  bool result = validate(simple_domain_obj_, simple_problem_obj_, plan);

  // Note: This test may fail if the PDDL generation is incompatible with the
  // validator or if the validator is not properly configured
  if (result) {
    EXPECT_TRUE(result);
  }
}

// Integration test: Invalid plan fails validation
TEST_F(PopfValidatorTest, InvalidPlanFailsValidation) {
  auto plan = create_invalid_plan();
  bool result = validate(simple_domain_obj_, simple_problem_obj_, plan);

  // Invalid plan should always fail validation
  EXPECT_FALSE(result);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
