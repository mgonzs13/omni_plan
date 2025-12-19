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
#include <set>
#include <string>
#include <thread>

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easy_plan/pddl/expression.hpp"
#include "easy_plan_knowledge_graph/kg_pddl_manager.hpp"

using namespace easy_plan_knowledge_graph;

// Global ROS initialization
class RosTestEnvironment : public ::testing::Environment {
public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

class KgPddlManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    kg_ = knowledge_graph::KnowledgeGraph::get_instance();
    manager_ = std::make_unique<KgPddlManager>();
  }

  void TearDown() override {
    manager_.reset();
    kg_.reset();
  }

  // Helper to create a node in the knowledge graph
  void create_node(const std::string &name, const std::string &type) {
    knowledge_graph::graph::Node node(name, type);
    kg_->update_node(node);
  }

  // Helper to create an edge in the knowledge graph
  void create_edge(const std::string &source, const std::string &target,
                   const std::string &type, bool is_goal = false) {
    knowledge_graph::graph::Edge edge(type, source, target);
    edge.set_property<bool>("is_goal", is_goal);
    kg_->update_edge(edge);
  }

  // Helper to create an Effect for testing
  easy_plan::pddl::Effect create_effect(const std::string &name,
                                        const std::vector<std::string> &args,
                                        bool negated = false) {
    auto predicate =
        std::make_shared<easy_plan::pddl::Predicate>(name, args, negated);
    easy_plan::pddl::Effect effect;
    effect.type = easy_plan::pddl::Effect::END;
    effect.expression = predicate;
    return effect;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
  std::unique_ptr<KgPddlManager> manager_;
};

// Test: Constructor creates manager
TEST_F(KgPddlManagerTest, ConstructorCreatesManager) {
  EXPECT_NE(manager_, nullptr);
}

// Test: Constructor with nullptr creates internal knowledge graph
TEST_F(KgPddlManagerTest, ConstructorWithNullptrCreatesInternalKg) {
  auto manager = std::make_unique<KgPddlManager>();
  EXPECT_NE(manager, nullptr);
}

// Test: get_pddl with empty knowledge graph
TEST_F(KgPddlManagerTest, GetPddlEmptyKnowledgeGraph) {
  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(domain.find("(define (domain knowledge_graph_domain)") !=
              std::string::npos);
  EXPECT_TRUE(problem.find("(define (problem knowledge_graph_problem)") !=
              std::string::npos);
}

// Test: get_pddl collects types from nodes
TEST_F(KgPddlManagerTest, GetPddlCollectsTypesFromNodes) {
  create_node("robot1", "robot");
  create_node("loc1", "location");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(domain.find("robot") != std::string::npos);
  EXPECT_TRUE(domain.find("location") != std::string::npos);
}

// Test: get_pddl includes action types
TEST_F(KgPddlManagerTest, GetPddlIncludesActionTypes) {
  std::set<std::string> types = {"custom_type", "another_type"};
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(domain.find("custom_type") != std::string::npos);
  EXPECT_TRUE(domain.find("another_type") != std::string::npos);
}

// Test: get_pddl collects predicates from edges
TEST_F(KgPddlManagerTest, GetPddlCollectsPredicatesFromEdges) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(domain.find("(at") != std::string::npos);
}

// Test: get_pddl includes action predicates
TEST_F(KgPddlManagerTest, GetPddlIncludesActionPredicates) {
  std::set<std::string> types;
  std::set<std::string> predicates = {"(custom_pred ?x - type)"};
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(domain.find("(custom_pred ?x - type)") != std::string::npos);
}

// Test: get_pddl includes actions
TEST_F(KgPddlManagerTest, GetPddlIncludesActions) {
  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions = {
      "(:action test_action :parameters () :precondition () :effect ())"};

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(domain.find("(:action test_action") != std::string::npos);
}

// Test: get_pddl generates objects from nodes
TEST_F(KgPddlManagerTest, GetPddlGeneratesObjectsFromNodes) {
  create_node("robot1", "robot");
  create_node("loc1", "location");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(problem.find("robot1 - robot") != std::string::npos);
  EXPECT_TRUE(problem.find("loc1 - location") != std::string::npos);
}

// Test: get_pddl generates init from edges (non-goal)
TEST_F(KgPddlManagerTest, GetPddlGeneratesInitFromEdges) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(problem.find("(:init") != std::string::npos);
  EXPECT_TRUE(problem.find("(at robot1 loc1)") != std::string::npos);
}

// Test: get_pddl handles self-referencing edges in init
TEST_F(KgPddlManagerTest, GetPddlHandlesSelfReferencingEdgesInInit) {
  create_node("robot1", "robot");
  create_edge("robot1", "robot1", "charging");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(problem.find("(charging robot1)") != std::string::npos);
}

// Test: get_pddl excludes goal edges from init
TEST_F(KgPddlManagerTest, GetPddlExcludesGoalEdgesFromInit) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_node("loc2", "location");
  create_edge("robot1", "loc1", "at");
  create_edge("robot1", "loc2", "at", true); // goal edge

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  // Init should contain at robot1 loc1 but not at robot1 loc2
  std::string init_section = problem.substr(
      problem.find("(:init"), problem.find("(:goal") - problem.find("(:init"));
  EXPECT_TRUE(init_section.find("(at robot1 loc1)") != std::string::npos);
  EXPECT_TRUE(init_section.find("(at robot1 loc2)") == std::string::npos);
}

// Test: get_pddl generates goals from goal edges
TEST_F(KgPddlManagerTest, GetPddlGeneratesGoalsFromGoalEdges) {
  create_node("robot1", "robot");
  create_node("loc2", "location");
  create_edge("robot1", "loc2", "at", true); // goal edge

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(problem.find("(:goal") != std::string::npos);
  EXPECT_TRUE(problem.find("( at robot1 loc2)") != std::string::npos);
}

// Test: get_pddl handles self-referencing edges in predicates
TEST_F(KgPddlManagerTest, GetPddlHandlesSelfReferencingEdgesInPredicates) {
  create_node("robot1", "robot");
  create_edge("robot1", "robot1", "charging");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  // Self-referencing predicate should have single parameter
  EXPECT_TRUE(domain.find("(charging ?r0 - robot)") != std::string::npos);
}

// Test: get_pddl domain has requirements
TEST_F(KgPddlManagerTest, GetPddlDomainHasRequirements) {
  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(domain.find(":typing") != std::string::npos);
  EXPECT_TRUE(domain.find(":durative-actions") != std::string::npos);
  EXPECT_TRUE(domain.find(":negative-preconditions") != std::string::npos);
}

// Test: apply_effect adds edge for positive effect
TEST_F(KgPddlManagerTest, ApplyEffectAddsEdgeForPositiveEffect) {
  create_node("robot1", "robot");
  create_node("loc1", "location");

  auto effect = create_effect("at", {"robot1", "loc1"}, false);
  manager_->apply_effect(effect);
  EXPECT_TRUE(this->kg_->has_edge("at", "robot1", "loc1"));
}

// Test: apply_effect removes edge for negative effect
TEST_F(KgPddlManagerTest, ApplyEffectRemovesEdgeForNegativeEffect) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");

  auto effect = create_effect("at", {"robot1", "loc1"}, true);
  manager_->apply_effect(effect);
  EXPECT_FALSE(this->kg_->has_edge("at", "robot1", "loc1"));
}

// Test: apply_effect handles single argument (self-referencing)
TEST_F(KgPddlManagerTest, ApplyEffectHandlesSingleArgument) {
  create_node("robot1", "robot");

  auto effect = create_effect("charging", {"robot1"}, false);
  manager_->apply_effect(effect);
  EXPECT_TRUE(this->kg_->has_edge("charging", "robot1", "robot1"));
}

// Test: Multiple nodes with same type
TEST_F(KgPddlManagerTest, MultipleNodesWithSameType) {
  create_node("robot1", "robot");
  create_node("robot2", "robot");
  create_node("loc1", "location");
  create_node("loc2", "location");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  // Types should appear only once
  size_t robot_count = 0;
  size_t pos = 0;
  std::string types_section = domain.substr(
      domain.find("(:types"), domain.find(")") - domain.find("(:types") + 1);
  while ((pos = types_section.find("robot", pos)) != std::string::npos) {
    robot_count++;
    pos += 5;
  }
  EXPECT_EQ(robot_count, 1u);

  // Objects should have all nodes
  EXPECT_TRUE(problem.find("robot1 - robot") != std::string::npos);
  EXPECT_TRUE(problem.find("robot2 - robot") != std::string::npos);
}

// Test: Multiple edges between nodes
TEST_F(KgPddlManagerTest, MultipleEdgesBetweenNodes) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");
  create_edge("robot1", "loc1", "near");

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  EXPECT_TRUE(problem.find("(at robot1 loc1)") != std::string::npos);
  EXPECT_TRUE(problem.find("(near robot1 loc1)") != std::string::npos);
}

// Test: Complex knowledge graph scenario
TEST_F(KgPddlManagerTest, ComplexKnowledgeGraphScenario) {
  // Set up a navigation scenario
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_node("loc2", "location");
  create_node("loc3", "location");

  create_edge("robot1", "loc1", "at");
  create_edge("loc1", "loc2", "connected");
  create_edge("loc2", "loc3", "connected");
  create_edge("robot1", "loc3", "at", true); // goal

  std::set<std::string> types;
  std::set<std::string> predicates;
  std::set<std::string> actions;

  auto [domain, problem] = manager_->get_pddl(types, predicates, actions);

  // Verify domain structure
  EXPECT_TRUE(domain.find("(:types") != std::string::npos);
  EXPECT_TRUE(domain.find("(:predicates") != std::string::npos);

  // Verify problem structure
  EXPECT_TRUE(problem.find("(:objects") != std::string::npos);
  EXPECT_TRUE(problem.find("(:init") != std::string::npos);
  EXPECT_TRUE(problem.find("(:goal") != std::string::npos);

  // Verify init state
  EXPECT_TRUE(problem.find("(at robot1 loc1)") != std::string::npos);
  EXPECT_TRUE(problem.find("(connected loc1 loc2)") != std::string::npos);
  EXPECT_TRUE(problem.find("(connected loc2 loc3)") != std::string::npos);

  // Verify goal
  EXPECT_TRUE(problem.find("( at robot1 loc3)") != std::string::npos);
}

// Test: Apply multiple effects sequentially
TEST_F(KgPddlManagerTest, ApplyMultipleEffectsSequentially) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_node("loc2", "location");
  create_edge("robot1", "loc1", "at");

  // Simulate moving from loc1 to loc2
  auto effect1 = create_effect("at", {"robot1", "loc1"}, true);  // not at loc1
  auto effect2 = create_effect("at", {"robot1", "loc2"}, false); // at loc2

  manager_->apply_effect(effect1);
  manager_->apply_effect(effect2);

  EXPECT_FALSE(this->kg_->has_edge("at", "robot1", "loc1"));
  EXPECT_TRUE(this->kg_->has_edge("at", "robot1", "loc2"));
}

// Test: get_pddl with no default parameters
TEST_F(KgPddlManagerTest, GetPddlNoParameters) {
  create_node("robot1", "robot");
  create_node("loc1", "location");

  auto [domain, problem] = manager_->get_pddl();

  EXPECT_TRUE(domain.find("(define (domain knowledge_graph_domain)") !=
              std::string::npos);
  EXPECT_TRUE(problem.find("(define (problem knowledge_graph_problem)") !=
              std::string::npos);
}

// =============================================================================
// predicate_exists tests
// =============================================================================

// Test: predicate_exists returns true when edge exists (two arguments)
TEST_F(KgPddlManagerTest, PredicateExistsReturnsTrueForExistingEdge) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");

  easy_plan::pddl::Predicate predicate("at", {"robot1", "loc1"});
  EXPECT_TRUE(manager_->predicate_exists(predicate));
}

// Test: predicate_exists returns false when edge does not exist
TEST_F(KgPddlManagerTest, PredicateExistsReturnsFalseForNonExistingEdge) {
  create_node("robot1", "robot");
  create_node("loc1", "location");

  easy_plan::pddl::Predicate predicate("at", {"robot1", "loc1"});
  EXPECT_FALSE(manager_->predicate_exists(predicate));
}

// Test: predicate_exists with single argument (self-referencing edge)
TEST_F(KgPddlManagerTest, PredicateExistsWithSingleArgument) {
  create_node("robot1", "robot");
  create_edge("robot1", "robot1", "charging");

  easy_plan::pddl::Predicate predicate("charging", {"robot1"});
  EXPECT_TRUE(manager_->predicate_exists(predicate));
}

// Test: predicate_exists returns false for wrong edge class
TEST_F(KgPddlManagerTest, PredicateExistsReturnsFalseForWrongEdgeClass) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");

  easy_plan::pddl::Predicate predicate("near", {"robot1", "loc1"});
  EXPECT_FALSE(manager_->predicate_exists(predicate));
}

// Test: predicate_exists with multiple edges between same nodes
TEST_F(KgPddlManagerTest, PredicateExistsWithMultipleEdges) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");
  create_edge("robot1", "loc1", "near");

  easy_plan::pddl::Predicate predicate_at("at", {"robot1", "loc1"});
  easy_plan::pddl::Predicate predicate_near("near", {"robot1", "loc1"});
  easy_plan::pddl::Predicate predicate_far("far", {"robot1", "loc1"});

  EXPECT_TRUE(manager_->predicate_exists(predicate_at));
  EXPECT_TRUE(manager_->predicate_exists(predicate_near));
  EXPECT_FALSE(manager_->predicate_exists(predicate_far));
}

// Test: predicate_exists after applying effect
TEST_F(KgPddlManagerTest, PredicateExistsAfterApplyingEffect) {
  create_node("robot1", "robot");
  create_node("loc1", "location");

  easy_plan::pddl::Predicate predicate("at", {"robot1", "loc1"});
  EXPECT_FALSE(manager_->predicate_exists(predicate));

  auto effect = create_effect("at", {"robot1", "loc1"}, false);
  manager_->apply_effect(effect);

  EXPECT_TRUE(manager_->predicate_exists(predicate));
}

// Test: predicate_exists after removing effect
TEST_F(KgPddlManagerTest, PredicateExistsAfterRemovingEffect) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at");

  easy_plan::pddl::Predicate predicate("at", {"robot1", "loc1"});
  EXPECT_TRUE(manager_->predicate_exists(predicate));

  auto effect = create_effect("at", {"robot1", "loc1"}, true);
  manager_->apply_effect(effect);

  EXPECT_FALSE(manager_->predicate_exists(predicate));
}

// =============================================================================
// predicate_is_goal tests
// =============================================================================

// Test: predicate_is_goal returns true for goal edge
TEST_F(KgPddlManagerTest, PredicateIsGoalReturnsTrueForGoalEdge) {
  create_node("robot1", "robot");
  create_node("loc2", "location");
  create_edge("robot1", "loc2", "at", true); // goal edge

  easy_plan::pddl::Predicate predicate("at", {"robot1", "loc2"});
  EXPECT_TRUE(manager_->predicate_is_goal(predicate));
}

// Test: predicate_is_goal returns false for non-goal edge
TEST_F(KgPddlManagerTest, PredicateIsGoalReturnsFalseForNonGoalEdge) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at", false); // not a goal edge

  easy_plan::pddl::Predicate predicate("at", {"robot1", "loc1"});
  EXPECT_FALSE(manager_->predicate_is_goal(predicate));
}

// Test: predicate_is_goal returns false when edge does not exist
TEST_F(KgPddlManagerTest, PredicateIsGoalReturnsFalseForNonExistingEdge) {
  create_node("robot1", "robot");
  create_node("loc1", "location");

  easy_plan::pddl::Predicate predicate("at", {"robot1", "loc1"});
  EXPECT_FALSE(manager_->predicate_is_goal(predicate));
}

// Test: predicate_is_goal with single argument (self-referencing goal edge)
TEST_F(KgPddlManagerTest, PredicateIsGoalWithSingleArgument) {
  create_node("robot1", "robot");
  create_edge("robot1", "robot1", "charged", true); // goal edge

  easy_plan::pddl::Predicate predicate("charged", {"robot1"});
  EXPECT_TRUE(manager_->predicate_is_goal(predicate));
}

// Test: predicate_is_goal with single argument non-goal
TEST_F(KgPddlManagerTest, PredicateIsGoalWithSingleArgumentNonGoal) {
  create_node("robot1", "robot");
  create_edge("robot1", "robot1", "charging", false); // not a goal edge

  easy_plan::pddl::Predicate predicate("charging", {"robot1"});
  EXPECT_FALSE(manager_->predicate_is_goal(predicate));
}

// Test: predicate_is_goal returns false for wrong edge class
TEST_F(KgPddlManagerTest, PredicateIsGoalReturnsFalseForWrongEdgeClass) {
  create_node("robot1", "robot");
  create_node("loc2", "location");
  create_edge("robot1", "loc2", "at", true); // goal edge

  easy_plan::pddl::Predicate predicate("near", {"robot1", "loc2"});
  EXPECT_FALSE(manager_->predicate_is_goal(predicate));
}

// Test: predicate_is_goal with multiple edges, one is goal
TEST_F(KgPddlManagerTest, PredicateIsGoalWithMultipleEdges) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_node("loc2", "location");
  create_edge("robot1", "loc1", "at", false); // init state
  create_edge("robot1", "loc2", "at", true);  // goal edge

  easy_plan::pddl::Predicate predicate_loc1("at", {"robot1", "loc1"});
  easy_plan::pddl::Predicate predicate_loc2("at", {"robot1", "loc2"});

  EXPECT_FALSE(manager_->predicate_is_goal(predicate_loc1));
  EXPECT_TRUE(manager_->predicate_is_goal(predicate_loc2));
}

// Test: predicate_is_goal with multiple edges same source/target different
// classes
TEST_F(KgPddlManagerTest, PredicateIsGoalWithDifferentEdgeClasses) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_edge("robot1", "loc1", "at", false);  // not a goal
  create_edge("robot1", "loc1", "near", true); // goal edge

  easy_plan::pddl::Predicate predicate_at("at", {"robot1", "loc1"});
  easy_plan::pddl::Predicate predicate_near("near", {"robot1", "loc1"});

  EXPECT_FALSE(manager_->predicate_is_goal(predicate_at));
  EXPECT_TRUE(manager_->predicate_is_goal(predicate_near));
}

// Test: predicate_exists and predicate_is_goal combined scenario
TEST_F(KgPddlManagerTest, PredicateExistsAndIsGoalCombinedScenario) {
  create_node("robot1", "robot");
  create_node("loc1", "location");
  create_node("loc2", "location");
  create_edge("robot1", "loc1", "at", false); // init position
  create_edge("robot1", "loc2", "at", true);  // goal position

  // Both predicates exist
  easy_plan::pddl::Predicate pred_loc1("at", {"robot1", "loc1"});
  easy_plan::pddl::Predicate pred_loc2("at", {"robot1", "loc2"});

  EXPECT_TRUE(manager_->predicate_exists(pred_loc1));
  EXPECT_TRUE(manager_->predicate_exists(pred_loc2));

  // Only loc2 is a goal
  EXPECT_FALSE(manager_->predicate_is_goal(pred_loc1));
  EXPECT_TRUE(manager_->predicate_is_goal(pred_loc2));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new RosTestEnvironment());
  return RUN_ALL_TESTS();
}
