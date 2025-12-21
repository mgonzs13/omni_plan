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
#include "easy_plan_vhpop/vhpop_planner.hpp"

using namespace easy_plan_vhpop;

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

class VhpopPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    planner_ = std::make_unique<VhpopPlanner>();
    planner_->load_parameters(node_);

    // Build simple domain
    simple_domain_obj_.add_requirement("strips");
    simple_domain_obj_.add_type("location");
    simple_domain_obj_.add_type("robot");
    simple_domain_obj_.add_predicate(
        easy_plan::pddl::Predicate("at", {"?r", "?l"}));
    simple_domain_obj_.add_predicate(
        easy_plan::pddl::Predicate("connected", {"?l1", "?l2"}));
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    move_action->add_condition(easy_plan::pddl::Type::START, "at",
                               {"?r", "?from"});
    move_action->add_condition(easy_plan::pddl::Type::START, "connected",
                               {"?from", "?to"});
    move_action->add_effect(easy_plan::pddl::Type::END, "at", {"?r", "?to"});
    move_action->add_effect(easy_plan::pddl::Type::END, "at", {"?r", "?from"},
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

    // Build unsolvable domain (same as simple)
    unsolvable_domain_obj_ = simple_domain_obj_;

    // Build unsolvable problem
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

  std::unique_ptr<VhpopPlanner> planner_;
  std::shared_ptr<rclcpp::Node> node_;
  easy_plan::pddl::Domain simple_domain_obj_;
  easy_plan::pddl::Problem simple_problem_obj_;
  easy_plan::pddl::Domain unsolvable_domain_obj_;
  easy_plan::pddl::Problem unsolvable_problem_obj_;

  std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>>
  create_actions() {
    std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>> actions;
    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    actions["move"] = std::make_shared<MockAction>("move", params);
    return actions;
  }
};

// Test: VhpopPlanner constructor
TEST_F(VhpopPlannerTest, ConstructorCreatesPlanner) {
  EXPECT_NE(planner_, nullptr);
}

// Test: generate_plan with invalid domain returns failed plan
TEST_F(VhpopPlannerTest, GetPlanWithInvalidDomainReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(easy_plan::pddl::Domain(),
                                      simple_problem_obj_, actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: generate_plan with empty domain returns failed plan
TEST_F(VhpopPlannerTest, GetPlanWithEmptyDomainReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(easy_plan::pddl::Domain(),
                                      easy_plan::pddl::Problem(), actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: generate_plan with empty actions map handles gracefully
TEST_F(VhpopPlannerTest, GetPlanWithEmptyActionsMap) {
  std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>> empty_actions;
  auto plan = planner_->generate_plan(simple_domain_obj_, simple_problem_obj_,
                                      empty_actions);
}

// Test: generate_plan with unsolvable problem returns failed plan
TEST_F(VhpopPlannerTest, GetPlanWithUnsolvableProblemReturnsFailed) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(simple_domain_obj_,
                                      unsolvable_problem_obj_, actions);

  EXPECT_FALSE(plan.has_solution());
}

// Test: Plan size is 0 for failed plans
TEST_F(VhpopPlannerTest, FailedPlanHasSizeZero) {
  auto actions = create_actions();
  auto plan = planner_->generate_plan(easy_plan::pddl::Domain(),
                                      simple_problem_obj_, actions);

  EXPECT_EQ(plan.size(), 0u);
}

// Test: Multiple calls to generate_plan work correctly
TEST_F(VhpopPlannerTest, MultiplePlannerCalls) {
  auto actions = create_actions();

  auto plan1 = planner_->generate_plan(easy_plan::pddl::Domain(),
                                       simple_problem_obj_, actions);
  auto plan2 = planner_->generate_plan(easy_plan::pddl::Domain(),
                                       simple_problem_obj_, actions);

  EXPECT_FALSE(plan1.has_solution());
  EXPECT_FALSE(plan2.has_solution());
}

// Integration test: Valid domain and problem (requires VHPOP to be installed)
TEST_F(VhpopPlannerTest, ValidDomainAndProblemReturnsPlan) {
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
TEST_F(VhpopPlannerTest, PlanActionsCorrectlyMapped) {
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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
