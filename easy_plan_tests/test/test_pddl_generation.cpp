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
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/pddl/timing_predicate.hpp"

using namespace easy_plan::pddl;

/**
 * @brief Mock action for PDDL generation tests
 */
class MockPddlAction : public Action {
public:
  MockPddlAction(const std::string &name,
                 std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params) {}

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    return ActionStatus::SUCCEED;
  }

  void cancel() override {}
};

/**
 * @brief Test fixture for PDDL generation tests
 */
class PddlGenerationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_pddl_generation_node");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
};

// ==================== Domain Tests ====================

/**
 * @brief Test domain with requirements
 */
TEST_F(PddlGenerationTest, DomainWithRequirements) {
  Domain domain;

  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_requirement("durative-actions");
  domain.add_requirement("negative-preconditions");

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(:requirements") != std::string::npos);
  EXPECT_TRUE(pddl.find(":strips") != std::string::npos);
  EXPECT_TRUE(pddl.find(":typing") != std::string::npos);
  EXPECT_TRUE(pddl.find(":durative-actions") != std::string::npos);
  EXPECT_TRUE(pddl.find(":negative-preconditions") != std::string::npos);
}

/**
 * @brief Test domain with types
 */
TEST_F(PddlGenerationTest, DomainWithTypes) {
  Domain domain;

  domain.add_type("robot");
  domain.add_type("location");
  domain.add_type("item");

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(:types") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("location") != std::string::npos);
  EXPECT_TRUE(pddl.find("item") != std::string::npos);
}

/**
 * @brief Test domain with predicates
 */
TEST_F(PddlGenerationTest, DomainWithPredicates) {
  Domain domain;

  domain.add_predicate(Predicate("at", {"robot", "location"}));
  domain.add_predicate(Predicate("holding", {"robot", "item"}));
  domain.add_predicate(Predicate("empty", {"robot"}));

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(:predicates") != std::string::npos);
  EXPECT_TRUE(pddl.find("at") != std::string::npos);
  EXPECT_TRUE(pddl.find("holding") != std::string::npos);
  EXPECT_TRUE(pddl.find("empty") != std::string::npos);
}

/**
 * @brief Test domain with actions
 */
TEST_F(PddlGenerationTest, DomainWithActions) {
  Domain domain;

  auto action = std::make_shared<MockPddlAction>(
      "pick", std::vector<std::pair<std::string, std::string>>{
                  {"r", "robot"}, {"i", "item"}, {"l", "location"}});
  action->add_condition(Type::START, "at", {"r", "l"});
  action->add_condition(Type::START, "empty", {"r"});
  action->add_effect(Type::END, "holding", {"r", "i"});
  action->add_effect(Type::END, "empty", {"r"}, true);

  domain.add_action(action);

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(:durative-action pick") != std::string::npos);
  EXPECT_TRUE(pddl.find(":parameters") != std::string::npos);
  EXPECT_TRUE(pddl.find(":condition") != std::string::npos);
  EXPECT_TRUE(pddl.find(":effect") != std::string::npos);
}

/**
 * @brief Test empty domain
 */
TEST_F(PddlGenerationTest, EmptyDomain) {
  Domain domain;

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(define (domain easy_plan_domain)") !=
              std::string::npos);
  EXPECT_TRUE(pddl.find("(:requirements)") != std::string::npos);
}

// ==================== Problem Tests ====================

/**
 * @brief Test problem with objects
 */
TEST_F(PddlGenerationTest, ProblemWithObjects) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("robot2", "robot"));
  problem.add_object(Object("loc_a", "location"));
  problem.add_object(Object("loc_b", "location"));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(:objects") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot1 - robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot2 - robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("loc_a - location") != std::string::npos);
  EXPECT_TRUE(pddl.find("loc_b - location") != std::string::npos);
}

/**
 * @brief Test problem with initial facts
 */
TEST_F(PddlGenerationTest, ProblemWithFacts) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc1", "location"));

  problem.add_fact(Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(Predicate("empty", {"robot1"}));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(:init") != std::string::npos);
  EXPECT_TRUE(pddl.find("(at robot1 loc1)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(empty robot1)") != std::string::npos);
}

/**
 * @brief Test problem with goals
 */
TEST_F(PddlGenerationTest, ProblemWithGoals) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc2", "location"));
  problem.add_object(Object("item1", "item"));

  problem.add_goal(Predicate("at", {"robot1", "loc2"}));
  problem.add_goal(Predicate("holding", {"robot1", "item1"}));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(:goal") != std::string::npos);
  EXPECT_TRUE(pddl.find("(and") != std::string::npos);
  EXPECT_TRUE(pddl.find("(at robot1 loc2)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(holding robot1 item1)") != std::string::npos);
}

/**
 * @brief Test complete problem
 */
TEST_F(PddlGenerationTest, CompleteProblem) {
  Problem problem;

  // Objects
  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc1", "location"));
  problem.add_object(Object("loc2", "location"));

  // Facts
  problem.add_fact(Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(Predicate("connected", {"loc1", "loc2"}));

  // Goals
  problem.add_goal(Predicate("at", {"robot1", "loc2"}));

  std::string pddl = problem.to_pddl();

  // Verify structure
  EXPECT_TRUE(pddl.find("(define (problem") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:domain easy_plan_domain)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:objects") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:init") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:goal") != std::string::npos);
}

// ==================== Predicate Tests ====================

/**
 * @brief Test predicate creation and properties
 */
TEST_F(PddlGenerationTest, PredicateCreation) {
  Predicate pred("at", {"robot", "location"}, false);

  EXPECT_EQ(pred.get_name(), "at");
  EXPECT_EQ(pred.get_args().size(), 2u);
  EXPECT_EQ(pred.get_args()[0], "robot");
  EXPECT_EQ(pred.get_args()[1], "location");
  EXPECT_FALSE(pred.is_negated());
}

/**
 * @brief Test negated predicate
 */
TEST_F(PddlGenerationTest, NegatedPredicate) {
  Predicate pred("holding", {"robot", "item"}, true);

  EXPECT_TRUE(pred.is_negated());

  std::string pddl = pred.to_pddl(true);
  EXPECT_TRUE(pddl.find("(not") != std::string::npos);
}

/**
 * @brief Test predicate negation toggle
 */
TEST_F(PddlGenerationTest, PredicateNegationToggle) {
  Predicate pred("empty", {"robot"});

  EXPECT_FALSE(pred.is_negated());

  pred.set_negation(true);
  EXPECT_TRUE(pred.is_negated());

  pred.set_negation(false);
  EXPECT_FALSE(pred.is_negated());
}

/**
 * @brief Test predicate comparison
 */
TEST_F(PddlGenerationTest, PredicateComparison) {
  Predicate pred1("at", {"robot", "location"});
  Predicate pred2("at", {"robot", "location"});
  Predicate pred3("holding", {"robot", "item"});

  // Same predicates should compare equal in terms of ordering
  EXPECT_FALSE(pred1 < pred2);
  EXPECT_FALSE(pred2 < pred1);

  // Different predicates should have ordering
  EXPECT_TRUE((pred1 < pred3) || (pred3 < pred1));
}

// ==================== Object Tests ====================

/**
 * @brief Test object creation
 */
TEST_F(PddlGenerationTest, ObjectCreation) {
  Object obj("robot1", "robot");

  EXPECT_EQ(obj.get_name(), "robot1");
  EXPECT_EQ(obj.get_type(), "robot");
}

/**
 * @brief Test object comparison
 */
TEST_F(PddlGenerationTest, ObjectComparison) {
  Object obj1("robot1", "robot");
  Object obj2("robot1", "robot");
  Object obj3("robot2", "robot");
  Object obj4("robot1", "vehicle");

  // Same objects
  EXPECT_FALSE(obj1 < obj2);
  EXPECT_FALSE(obj2 < obj1);

  // Different names
  EXPECT_TRUE((obj1 < obj3) || (obj3 < obj1));

  // Same name, different type
  EXPECT_TRUE((obj1 < obj4) || (obj4 < obj1));
}

// ==================== TimingPredicate Tests ====================

/**
 * @brief Test timing predicate with START type
 */
TEST_F(PddlGenerationTest, TimingPredicateStart) {
  TimingPredicate pred(Type::START, "at", {"r", "l"});

  EXPECT_EQ(pred.get_type(), Type::START);
  EXPECT_EQ(pred.get_name(), "at");

  std::string pddl = pred.to_pddl();
  EXPECT_TRUE(pddl.find("at start") != std::string::npos);
}

/**
 * @brief Test timing predicate with OVER_ALL type
 */
TEST_F(PddlGenerationTest, TimingPredicateOverAll) {
  TimingPredicate pred(Type::OVER_ALL, "moving", {"r"});

  EXPECT_EQ(pred.get_type(), Type::OVER_ALL);

  std::string pddl = pred.to_pddl();
  EXPECT_TRUE(pddl.find("over all") != std::string::npos);
}

/**
 * @brief Test timing predicate with END type
 */
TEST_F(PddlGenerationTest, TimingPredicateEnd) {
  TimingPredicate pred(Type::END, "at", {"r", "l"});

  EXPECT_EQ(pred.get_type(), Type::END);

  std::string pddl = pred.to_pddl();
  EXPECT_TRUE(pddl.find("at end") != std::string::npos);
}

/**
 * @brief Test negated timing predicate
 */
TEST_F(PddlGenerationTest, NegatedTimingPredicate) {
  TimingPredicate pred(Type::START, "at", {"r", "l"}, true);

  EXPECT_TRUE(pred.is_negated());

  std::string pddl = pred.to_pddl();
  EXPECT_TRUE(pddl.find("(not") != std::string::npos);
}

// ==================== Action Tests ====================

/**
 * @brief Test action parameter retrieval
 */
TEST_F(PddlGenerationTest, ActionParameters) {
  auto action = std::make_shared<MockPddlAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"r", "robot"}, {"from", "location"}, {"to", "location"}});

  auto params = action->get_parameters();
  EXPECT_EQ(params.size(), 3u);

  EXPECT_EQ(action->get_parameter_type("r"), "robot");
  EXPECT_EQ(action->get_parameter_type("from"), "location");
  EXPECT_EQ(action->get_parameter_type("to"), "location");
  EXPECT_EQ(action->get_parameter_type("unknown"), "unknown_type");

  EXPECT_EQ(action->get_parameter_index("r"), 0);
  EXPECT_EQ(action->get_parameter_index("from"), 1);
  EXPECT_EQ(action->get_parameter_index("to"), 2);
  EXPECT_EQ(action->get_parameter_index("unknown"), -1);
}

/**
 * @brief Test action conditions
 */
TEST_F(PddlGenerationTest, ActionConditions) {
  auto action = std::make_shared<MockPddlAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"r", "robot"}, {"from", "location"}, {"to", "location"}});

  action->add_condition(Type::START, "at", {"r", "from"});
  action->add_condition(Type::START, "connected", {"from", "to"});
  action->add_condition(Type::OVER_ALL, "available", {"r"});
  action->add_condition(Type::END, "clear", {"to"});

  EXPECT_EQ(action->get_conditions().size(), 4u);
  EXPECT_EQ(action->get_on_start_conditions().size(), 2u);
  EXPECT_EQ(action->get_over_all_conditions().size(), 1u);
  EXPECT_EQ(action->get_on_end_conditions().size(), 1u);
}

/**
 * @brief Test action effects
 */
TEST_F(PddlGenerationTest, ActionEffects) {
  auto action = std::make_shared<MockPddlAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"r", "robot"}, {"from", "location"}, {"to", "location"}});

  action->add_effect(Type::START, "at", {"r", "from"}, true);
  action->add_effect(Type::END, "at", {"r", "to"});
  action->add_effect(Type::OVER_ALL, "moving", {"r"});

  EXPECT_EQ(action->get_effects().size(), 3u);
  EXPECT_EQ(action->get_on_start_effects().size(), 1u);
  EXPECT_EQ(action->get_over_all_effects().size(), 1u);
  EXPECT_EQ(action->get_on_end_effects().size(), 1u);
}

/**
 * @brief Test action PDDL generation
 */
TEST_F(PddlGenerationTest, ActionToPddl) {
  auto action = std::make_shared<MockPddlAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"r", "robot"}, {"from", "location"}, {"to", "location"}});

  action->add_condition(Type::START, "at", {"r", "from"});
  action->add_effect(Type::END, "at", {"r", "to"});

  std::string pddl = action->to_pddl();

  EXPECT_TRUE(pddl.find("(:durative-action move") != std::string::npos);
  EXPECT_TRUE(pddl.find(":parameters") != std::string::npos);
  EXPECT_TRUE(pddl.find(":duration") != std::string::npos);
  EXPECT_TRUE(pddl.find(":condition") != std::string::npos);
  EXPECT_TRUE(pddl.find(":effect") != std::string::npos);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
