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
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"

using namespace easy_plan::pddl;

/**
 * @brief Mock action for integration testing
 */
class TestMoveAction : public Action {
public:
  TestMoveAction()
      : Action("move",
               {{"r", "robot"}, {"from", "location"}, {"to", "location"}}) {
    // At start conditions: robot is at 'from' and locations are connected
    add_condition(Type::START, "at", {"r", "from"});
    add_condition(Type::START, "connected", {"from", "to"});

    // At end effects: robot moves to 'to' and leaves 'from'
    add_effect(Type::START, "at", {"r", "from"}, true);
    add_effect(Type::END, "at", {"r", "to"});
  }

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    return ActionStatus::SUCCEED;
  }

  void cancel() override {}
};

/**
 * @brief Mock action for charging robot
 */
class TestChargeAction : public Action {
public:
  TestChargeAction() : Action("charge", {{"r", "robot"}, {"loc", "charger"}}) {
    // Conditions: robot is at the charger location
    add_condition(Type::START, "at", {"r", "loc"});
    add_condition(Type::START, "is_charger", {"loc"});

    // Effects: robot becomes charged
    add_effect(Type::END, "charged", {"r"});
  }

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    return ActionStatus::SUCCEED;
  }

  void cancel() override {}
};

/**
 * @brief Test fixture for planning pipeline integration tests
 */
class PlanningPipelineTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_planning_pipeline_node");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
};

/**
 * @brief Test creating a complete domain with multiple actions
 */
TEST_F(PlanningPipelineTest, CreateCompleteDomain) {
  Domain domain;

  // Add requirements
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_requirement("durative-actions");

  // Add types
  domain.add_type("robot");
  domain.add_type("location");
  domain.add_type("charger");

  // Add predicates
  domain.add_predicate(Predicate("at", {"robot", "location"}));
  domain.add_predicate(Predicate("connected", {"location", "location"}));
  domain.add_predicate(Predicate("charged", {"robot"}));
  domain.add_predicate(Predicate("is_charger", {"charger"}));

  // Add actions
  auto move_action = std::make_shared<TestMoveAction>();
  auto charge_action = std::make_shared<TestChargeAction>();
  domain.add_action(move_action);
  domain.add_action(charge_action);

  // Generate PDDL
  std::string pddl = domain.to_pddl();

  // Verify PDDL content
  EXPECT_TRUE(pddl.find("(define (domain easy_plan_domain)") !=
              std::string::npos);
  EXPECT_TRUE(pddl.find(":strips") != std::string::npos);
  EXPECT_TRUE(pddl.find(":typing") != std::string::npos);
  EXPECT_TRUE(pddl.find(":durative-actions") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:types") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("location") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:predicates") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:durative-action move") != std::string::npos ||
              pddl.find("(:durative-action charge") != std::string::npos);
}

/**
 * @brief Test creating a problem with objects, facts, and goals
 */
TEST_F(PlanningPipelineTest, CreateCompleteProblem) {
  Problem problem;

  // Add objects
  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("robot2", "robot"));
  problem.add_object(Object("loc1", "location"));
  problem.add_object(Object("loc2", "location"));
  problem.add_object(Object("loc3", "location"));
  problem.add_object(Object("charger1", "charger"));

  // Add initial facts
  problem.add_fact(Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(Predicate("at", {"robot2", "loc2"}));
  problem.add_fact(Predicate("connected", {"loc1", "loc2"}));
  problem.add_fact(Predicate("connected", {"loc2", "loc3"}));
  problem.add_fact(Predicate("is_charger", {"charger1"}));

  // Add goals
  problem.add_goal(Predicate("at", {"robot1", "loc3"}));
  problem.add_goal(Predicate("charged", {"robot1"}));

  // Generate PDDL
  std::string pddl = problem.to_pddl();

  // Verify PDDL content
  EXPECT_TRUE(pddl.find("(define (problem") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:domain easy_plan_domain)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:objects") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot1 - robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot2 - robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:init") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:goal") != std::string::npos);
}

/**
 * @brief Test creating and manipulating a plan
 */
TEST_F(PlanningPipelineTest, CreateAndManipulatePlan) {
  Plan plan(true);

  auto move_action = std::make_shared<TestMoveAction>();
  auto charge_action = std::make_shared<TestChargeAction>();

  // Add actions to plan
  plan.add_action(move_action, {"robot1", "loc1", "loc2"});
  plan.add_action(move_action, {"robot1", "loc2", "loc3"});
  plan.add_action(charge_action, {"robot1", "charger1"});

  // Verify plan properties
  EXPECT_TRUE(plan.has_solution());
  EXPECT_EQ(plan.size(), 3u);

  // Verify action retrieval
  EXPECT_EQ(plan.get_action(0)->get_name(), "move");
  EXPECT_EQ(plan.get_action(1)->get_name(), "move");
  EXPECT_EQ(plan.get_action(2)->get_name(), "charge");

  // Verify parameters
  auto params0 = plan.get_action_params(0);
  EXPECT_EQ(params0.size(), 3u);
  EXPECT_EQ(params0[0], "robot1");
  EXPECT_EQ(params0[1], "loc1");
  EXPECT_EQ(params0[2], "loc2");

  // Verify action with params
  auto [action, params] = plan.get_action_with_params(1);
  EXPECT_EQ(action->get_name(), "move");
  EXPECT_EQ(params.size(), 3u);
}

/**
 * @brief Test plan PDDL generation
 */
TEST_F(PlanningPipelineTest, PlanToPddl) {
  Plan plan(true);

  auto move_action = std::make_shared<TestMoveAction>();

  plan.add_action(move_action, {"robot1", "loc1", "loc2"});
  plan.add_action(move_action, {"robot1", "loc2", "loc3"});

  std::string pddl = plan.to_pddl();

  EXPECT_TRUE(pddl.find("(move robot1 loc1 loc2)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(move robot1 loc2 loc3)") != std::string::npos);
}

/**
 * @brief Test action execution through plan
 */
TEST_F(PlanningPipelineTest, ExecutePlanActions) {
  Plan plan(true);

  auto move_action = std::make_shared<TestMoveAction>();
  plan.add_action(move_action, {"robot1", "loc1", "loc2"});

  // Execute action
  auto [action, params] = plan.get_action_with_params(0);
  ActionStatus status = action->run(params);

  EXPECT_EQ(status, ActionStatus::SUCCEED);
}

/**
 * @brief Test building complete domain and problem from actions
 */
TEST_F(PlanningPipelineTest, BuildDomainAndProblemFromActions) {
  // Create actions
  std::vector<std::shared_ptr<Action>> actions;
  actions.push_back(std::make_shared<TestMoveAction>());
  actions.push_back(std::make_shared<TestChargeAction>());

  // Create action map
  std::unordered_map<std::string, std::shared_ptr<Action>> action_map;
  for (const auto &action : actions) {
    action_map[action->get_name()] = action;
  }

  // Verify action map
  EXPECT_EQ(action_map.size(), 2u);
  EXPECT_NE(action_map.find("move"), action_map.end());
  EXPECT_NE(action_map.find("charge"), action_map.end());

  // Verify action properties
  auto move = action_map["move"];
  EXPECT_EQ(move->get_parameters().size(), 3u);
  EXPECT_EQ(move->get_conditions().size(), 2u);
  EXPECT_EQ(move->get_effects().size(), 2u);

  auto charge = action_map["charge"];
  EXPECT_EQ(charge->get_parameters().size(), 2u);
  EXPECT_EQ(charge->get_conditions().size(), 2u);
  EXPECT_EQ(charge->get_effects().size(), 1u);
}

/**
 * @brief Test empty plan behavior
 */
TEST_F(PlanningPipelineTest, EmptyPlanBehavior) {
  Plan plan(false);

  EXPECT_FALSE(plan.has_solution());
  EXPECT_EQ(plan.size(), 0u);

  std::string pddl = plan.to_pddl();
  EXPECT_EQ(pddl, "\n");
}

/**
 * @brief Test plan with no solution
 */
TEST_F(PlanningPipelineTest, PlanWithNoSolution) {
  Plan plan;

  EXPECT_FALSE(plan.has_solution());
  plan.set_has_solution(true);
  EXPECT_TRUE(plan.has_solution());
  plan.set_has_solution(false);
  EXPECT_FALSE(plan.has_solution());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
