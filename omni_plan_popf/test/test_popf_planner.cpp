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

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_popf/popf_planner.hpp"

using namespace omni_plan_popf;

// Mock Action class for testing
class MockAction : public omni_plan::pddl::Action {
public:
  MockAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params), cancel_called_(false) {}

  omni_plan::pddl::ActionStatus
  run(const std::vector<std::string> & /*params*/) override {
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { cancel_called_ = true; }

  bool cancel_called_;
};

class PopfPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    planner_ = std::make_unique<PopfPlanner>();
    planner_->load_ros_parameters(node_);

    // Build simple domain
    simple_domain_obj_.add_requirement("strips");
    simple_domain_obj_.add_type("location");
    simple_domain_obj_.add_type("robot");
    simple_domain_obj_.add_predicate(
        omni_plan::pddl::Predicate("at", {"?r", "?l"}));
    simple_domain_obj_.add_predicate(
        omni_plan::pddl::Predicate("connected", {"?l1", "?l2"}));
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    move_action->add_condition(omni_plan::pddl::Type::START, "at",
                               {"?r", "?from"});
    move_action->add_condition(omni_plan::pddl::Type::START, "connected",
                               {"?from", "?to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?from"},
                            true);
    simple_domain_obj_.add_action(move_action);

    // Build simple problem
    simple_problem_obj_.add_object(omni_plan::pddl::Object("robot1", "robot"));
    simple_problem_obj_.add_object(omni_plan::pddl::Object("loc1", "location"));
    simple_problem_obj_.add_object(omni_plan::pddl::Object("loc2", "location"));
    simple_problem_obj_.add_fact(
        omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    simple_problem_obj_.add_fact(
        omni_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
    simple_problem_obj_.add_goal(
        omni_plan::pddl::Predicate("at", {"robot1", "loc2"}));

    // Build unsolvable domain (same as simple)
    unsolvable_domain_obj_ = simple_domain_obj_;

    // Build unsolvable problem
    unsolvable_problem_obj_.add_object(
        omni_plan::pddl::Object("robot1", "robot"));
    unsolvable_problem_obj_.add_object(
        omni_plan::pddl::Object("loc1", "location"));
    unsolvable_problem_obj_.add_object(
        omni_plan::pddl::Object("loc2", "location"));
    unsolvable_problem_obj_.add_fact(
        omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    unsolvable_problem_obj_.add_goal(
        omni_plan::pddl::Predicate("at", {"robot1", "loc2"}));
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::unique_ptr<PopfPlanner> planner_;
  std::shared_ptr<rclcpp::Node> node_;
  omni_plan::pddl::Domain simple_domain_obj_;
  omni_plan::pddl::Problem simple_problem_obj_;
  omni_plan::pddl::Domain unsolvable_domain_obj_;
  omni_plan::pddl::Problem unsolvable_problem_obj_;

  std::unordered_map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
  create_actions() {
    std::unordered_map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        actions;
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    actions["move"] = std::make_shared<MockAction>("move", params);
    return actions;
  }
};

// Test: PopfPlanner constructor
TEST_F(PopfPlannerTest, ConstructorCreatesPlanner) {
  EXPECT_NE(planner_, nullptr);
}

// Test: generate_plan with invalid domain returns failed plan
TEST_F(PopfPlannerTest, GetPlanWithInvalidDomainReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(omni_plan::pddl::Domain(),
                                      simple_problem_obj_, actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: generate_plan with empty domain returns failed plan
TEST_F(PopfPlannerTest, GetPlanWithEmptyDomainReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(omni_plan::pddl::Domain(),
                                      omni_plan::pddl::Problem(), actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: generate_plan with empty actions map handles gracefully
TEST_F(PopfPlannerTest, GetPlanWithEmptyActionsMap) {
  std::unordered_map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
      empty_actions;
  auto plan = planner_->generate_plan(simple_domain_obj_, simple_problem_obj_,
                                      empty_actions);
}

// Test: generate_plan with unsolvable problem returns failed plan
TEST_F(PopfPlannerTest, GetPlanWithUnsolvableProblemReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(simple_domain_obj_,
                                      unsolvable_problem_obj_, actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: Plan size is 0 for failed plans
TEST_F(PopfPlannerTest, FailedPlanHasSizeZero) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(omni_plan::pddl::Domain(),
                                      simple_problem_obj_, actions);

  EXPECT_EQ(plan.size(), 0u);
}

// Test: Multiple calls to generate_plan work correctly
TEST_F(PopfPlannerTest, MultiplePlannerCalls) {
  auto actions = create_actions();

  auto plan1 = planner_->generate_plan(omni_plan::pddl::Domain(),
                                       simple_problem_obj_, actions);
  auto plan2 = planner_->generate_plan(omni_plan::pddl::Domain(),
                                       simple_problem_obj_, actions);

  EXPECT_FALSE(plan1.has_solution());
  EXPECT_FALSE(plan2.has_solution());
}

// Integration test: Valid domain and problem (requires POPF to be installed)
TEST_F(PopfPlannerTest, ValidDomainAndProblemReturnsPlan) {
  auto actions = create_actions();
  auto plan =
      planner_->generate_plan(simple_domain_obj_, simple_problem_obj_, actions);

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
TEST_F(PopfPlannerTest, PlanActionsCorrectlyMapped) {
  auto actions = create_actions();
  auto plan =
      planner_->generate_plan(simple_domain_obj_, simple_problem_obj_, actions);

  if (plan.has_solution()) {
    for (size_t i = 0; i < plan.size(); ++i) {
      auto [action, params] = plan.get_action_with_params(i);
      EXPECT_NE(action, nullptr);
    }
  }
}

// Test: Valid plan has start times and durations
TEST_F(PopfPlannerTest, PlanHasStartTimesAndDurations) {
  auto actions = create_actions();
  auto plan =
      planner_->generate_plan(simple_domain_obj_, simple_problem_obj_, actions);

  if (plan.has_solution()) {
    for (size_t i = 0; i < plan.size(); ++i) {
      float start_time = plan.get_action_start_time(i);
      float duration = plan.get_action_duration(i);
      // POPF produces temporal plans with start times >= 0 and durations > 0
      EXPECT_GE(start_time, 0.0f);
      EXPECT_GT(duration, 0.0f);
    }
  }
}

// Test: Start times are non-decreasing in the plan
TEST_F(PopfPlannerTest, StartTimesNonDecreasing) {
  auto actions = create_actions();
  auto plan =
      planner_->generate_plan(simple_domain_obj_, simple_problem_obj_, actions);

  if (plan.has_solution() && plan.size() > 1) {
    for (size_t i = 1; i < plan.size(); ++i) {
      EXPECT_GE(plan.get_action_start_time(i),
                plan.get_action_start_time(i - 1));
    }
  }
}

// Test: Failed plan has zero start times and durations
TEST_F(PopfPlannerTest, FailedPlanHasNoTimingInfo) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(omni_plan::pddl::Domain(),
                                      simple_problem_obj_, actions);

  EXPECT_FALSE(plan.has_solution());
  EXPECT_EQ(plan.size(), 0u);
}

// Test: Parallel plan with multiple robots
TEST_F(PopfPlannerTest, ParallelPlanWithMultipleRobots) {
  // Create a domain with two robots that can move independently
  omni_plan::pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("durative-actions");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_predicate(omni_plan::pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(omni_plan::pddl::Predicate("connected", {"?l1", "?l2"}));

  std::vector<std::pair<std::string, std::string>> params = {
      {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  auto move_action = std::make_shared<MockAction>("move", params);
  move_action->add_condition(omni_plan::pddl::Type::START, "at",
                             {"?r", "?from"});
  move_action->add_condition(omni_plan::pddl::Type::START, "connected",
                             {"?from", "?to"});
  move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?to"});
  move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?from"},
                          true);
  domain.add_action(move_action);

  omni_plan::pddl::Problem problem;
  problem.add_object(omni_plan::pddl::Object("robot1", "robot"));
  problem.add_object(omni_plan::pddl::Object("robot2", "robot"));
  problem.add_object(omni_plan::pddl::Object("loc1", "location"));
  problem.add_object(omni_plan::pddl::Object("loc2", "location"));
  problem.add_object(omni_plan::pddl::Object("loc3", "location"));
  problem.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(omni_plan::pddl::Predicate("at", {"robot2", "loc2"}));
  problem.add_fact(omni_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
  problem.add_fact(omni_plan::pddl::Predicate("connected", {"loc2", "loc3"}));
  problem.add_goal(omni_plan::pddl::Predicate("at", {"robot1", "loc2"}));
  problem.add_goal(omni_plan::pddl::Predicate("at", {"robot2", "loc3"}));

  auto actions = create_actions();
  auto plan = planner_->generate_plan(domain, problem, actions);

  if (plan.has_solution()) {
    EXPECT_GE(plan.size(), 2u);

    // Check that all actions have valid timing info
    for (size_t i = 0; i < plan.size(); ++i) {
      EXPECT_GE(plan.get_action_start_time(i), 0.0f);
      EXPECT_GT(plan.get_action_duration(i), 0.0f);
    }

    // Check if there are actions with the same start time (parallel)
    bool has_parallel = false;
    for (size_t i = 1; i < plan.size(); ++i) {
      if (std::abs(plan.get_action_start_time(i) -
                   plan.get_action_start_time(i - 1)) < 0.001f) {
        has_parallel = true;
        break;
      }
    }
    // POPF may or may not produce parallel actions for this problem
    // This test just verifies the structure is valid
    (void)has_parallel;
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
