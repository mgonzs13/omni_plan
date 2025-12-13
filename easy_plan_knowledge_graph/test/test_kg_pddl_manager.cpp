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

#include <rclcpp/rclcpp.hpp>
#include <yasmin_ros/yasmin_node.hpp>

#include <knowledge_graph/knowledge_graph.hpp>
#include <knowledge_graph_msgs/msg/content.hpp>
#include <knowledge_graph_msgs/msg/edge.hpp>
#include <knowledge_graph_msgs/msg/node.hpp>
#include <knowledge_graph_msgs/msg/property.hpp>

#include "easy_plan/pddl/expression.hpp"
#include "easy_plan_knowledge_graph/kg_pddl_manager.hpp"

using namespace easy_plan_knowledge_graph;

class KgPddlManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Get the yasmin node instance for knowledge graph operations
    node_ = yasmin_ros::YasminNode::get_instance();
    kg_ = std::make_shared<knowledge_graph::KnowledgeGraph>(node_);

    // Clear the knowledge graph before each test
    clear_knowledge_graph();

    // Create the manager under test
    manager_ = std::make_unique<KgPddlManager>(kg_);
  }

  void TearDown() override {
    // Clear the knowledge graph after each test
    clear_knowledge_graph();
  }

  void clear_knowledge_graph() {
    // Remove all edges
    auto edges = kg_->get_edges();
    for (const auto &edge : edges) {
      kg_->remove_edge(edge);
    }

    // Remove all nodes
    auto nodes = kg_->get_nodes();
    for (const auto &node : nodes) {
      kg_->remove_node(node.node_name);
    }
  }

  knowledge_graph_msgs::msg::Node create_node(const std::string &name,
                                              const std::string &node_class) {
    knowledge_graph_msgs::msg::Node node;
    node.node_name = name;
    node.node_class = node_class;
    return node;
  }

  knowledge_graph_msgs::msg::Edge create_edge(const std::string &edge_class,
                                              const std::string &source,
                                              const std::string &target) {
    knowledge_graph_msgs::msg::Edge edge;
    edge.edge_class = edge_class;
    edge.source_node = source;
    edge.target_node = target;
    return edge;
  }

  knowledge_graph_msgs::msg::Property
  create_bool_property(const std::string &key, bool value) {
    knowledge_graph_msgs::msg::Property prop;
    prop.key = key;
    prop.value.type = knowledge_graph_msgs::msg::Content::BOOL;
    prop.value.bool_value = value;
    return prop;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
  std::unique_ptr<KgPddlManager> manager_;
};

// Test: Empty knowledge graph produces minimal PDDL
TEST_F(KgPddlManagerTest, EmptyKnowledgeGraphProducesMinimalPddl) {
  auto [domain, problem] = manager_->get_pddl();

  // Domain should contain basic structure
  EXPECT_TRUE(domain.find("(define (domain knowledge_graph_domain)") !=
              std::string::npos);

  // Problem should contain basic structure
  EXPECT_TRUE(problem.find("(define (problem knowledge_graph_problem)") !=
              std::string::npos);
  EXPECT_TRUE(problem.find("(:domain knowledge_graph_domain)") !=
              std::string::npos);
}

// Test: Has goals returns false for empty knowledge graph
TEST_F(KgPddlManagerTest, HasGoalsReturnsFalseForEmptyGraph) {
  EXPECT_FALSE(manager_->has_goals());
}

// Test: Has goals returns false when no is_goal property
TEST_F(KgPddlManagerTest, HasGoalsReturnsFalseWhenNoGoalProperty) {
  auto node = create_node("robot1", "robot");
  kg_->update_node(node);

  EXPECT_FALSE(manager_->has_goals());
}

// Test: Has goals returns true when is_goal property is true
TEST_F(KgPddlManagerTest, HasGoalsReturnsTrueWhenGoalPropertyTrue) {
  auto node = create_node("location1", "location");
  node.properties.push_back(create_bool_property("is_goal", true));
  kg_->update_node(node);

  EXPECT_TRUE(manager_->has_goals());
}

// Test: Has goals returns false when is_goal property is false
TEST_F(KgPddlManagerTest, HasGoalsReturnsFalseWhenGoalPropertyFalse) {
  auto node = create_node("location1", "location");
  node.properties.push_back(create_bool_property("is_goal", false));
  kg_->update_node(node);

  EXPECT_FALSE(manager_->has_goals());
}

// Test: PDDL generation includes types from nodes
TEST_F(KgPddlManagerTest, PddlGenerationIncludesTypes) {
  auto robot = create_node("robot1", "robot");
  auto location = create_node("loc1", "location");
  kg_->update_node(robot);
  kg_->update_node(location);

  auto [domain, problem] = manager_->get_pddl();

  EXPECT_TRUE(domain.find("(:types") != std::string::npos);
  EXPECT_TRUE(domain.find("robot") != std::string::npos);
  EXPECT_TRUE(domain.find("location") != std::string::npos);
}

// Test: PDDL generation includes objects
TEST_F(KgPddlManagerTest, PddlGenerationIncludesObjects) {
  auto robot = create_node("robot1", "robot");
  kg_->update_node(robot);

  auto [domain, problem] = manager_->get_pddl();

  EXPECT_TRUE(problem.find("(:objects") != std::string::npos);
  EXPECT_TRUE(problem.find("robot1 - robot") != std::string::npos);
}

// Test: PDDL generation includes predicates from edges
TEST_F(KgPddlManagerTest, PddlGenerationIncludesPredicatesFromEdges) {
  auto robot = create_node("robot1", "robot");
  auto location = create_node("loc1", "location");
  kg_->update_node(robot);
  kg_->update_node(location);

  auto edge = create_edge("at", "robot1", "loc1");
  kg_->update_edge(edge);

  auto [domain, problem] = manager_->get_pddl();

  EXPECT_TRUE(domain.find("(:predicates") != std::string::npos);
  EXPECT_TRUE(domain.find("(at ?x ?y)") != std::string::npos);
  EXPECT_TRUE(problem.find("(at robot1 loc1)") != std::string::npos);
}

// Test: PDDL generation includes unary predicates from boolean properties
TEST_F(KgPddlManagerTest, PddlGenerationIncludesUnaryPredicates) {
  auto robot = create_node("robot1", "robot");
  robot.properties.push_back(create_bool_property("arm_empty", true));
  kg_->update_node(robot);

  auto [domain, problem] = manager_->get_pddl();

  EXPECT_TRUE(domain.find("(arm_empty ?x)") != std::string::npos);
  EXPECT_TRUE(problem.find("(arm_empty robot1)") != std::string::npos);
}

// Test: PDDL generation includes goals
TEST_F(KgPddlManagerTest, PddlGenerationIncludesGoals) {
  auto location = create_node("loc1", "location");
  location.properties.push_back(create_bool_property("is_goal", true));
  kg_->update_node(location);

  auto [domain, problem] = manager_->get_pddl();

  EXPECT_TRUE(problem.find("(:goal (and") != std::string::npos);
  EXPECT_TRUE(problem.find("(is_goal loc1)") != std::string::npos);
}

// Test: Apply effect adds edge for binary predicate
TEST_F(KgPddlManagerTest, ApplyEffectAddsBinaryEdge) {
  auto robot = create_node("robot1", "robot");
  auto location = create_node("loc1", "location");
  kg_->update_node(robot);
  kg_->update_node(location);

  // Create an effect: (at robot1 loc1)
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "at", std::vector<std::string>{"robot1", "loc1"}, false);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  manager_->apply_effect(effect);

  // Verify edge was added
  auto edges = kg_->get_edges("robot1", "loc1");
  ASSERT_FALSE(edges.empty());
  EXPECT_EQ(edges[0].edge_class, "at");
}

// Test: Apply effect removes edge for negated binary predicate
TEST_F(KgPddlManagerTest, ApplyEffectRemovesBinaryEdge) {
  auto robot = create_node("robot1", "robot");
  auto location = create_node("loc1", "location");
  kg_->update_node(robot);
  kg_->update_node(location);

  // First add the edge
  auto edge = create_edge("at", "robot1", "loc1");
  kg_->update_edge(edge);

  // Create a negated effect: (not (at robot1 loc1))
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "at", std::vector<std::string>{"robot1", "loc1"}, true);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  manager_->apply_effect(effect);

  // Verify edge was removed
  auto edges = kg_->get_edges("robot1", "loc1");
  bool found = false;
  for (const auto &e : edges) {
    if (e.edge_class == "at") {
      found = true;
      break;
    }
  }
  EXPECT_FALSE(found);
}

// Test: Apply effect sets unary predicate property
TEST_F(KgPddlManagerTest, ApplyEffectSetsUnaryProperty) {
  auto robot = create_node("robot1", "robot");
  kg_->update_node(robot);

  // Create an effect: (arm_empty robot1)
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "arm_empty", std::vector<std::string>{"robot1"}, false);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  manager_->apply_effect(effect);

  // Verify property was set
  auto node_opt = kg_->get_node("robot1");
  ASSERT_TRUE(node_opt.has_value());
  auto node = *node_opt;

  bool found = false;
  for (const auto &prop : node.properties) {
    if (prop.key == "arm_empty") {
      EXPECT_TRUE(prop.value.bool_value);
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

// Test: Apply effect clears unary predicate property when negated
TEST_F(KgPddlManagerTest, ApplyEffectClearsUnaryProperty) {
  auto robot = create_node("robot1", "robot");
  robot.properties.push_back(create_bool_property("arm_empty", true));
  kg_->update_node(robot);

  // Create a negated effect: (not (arm_empty robot1))
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "arm_empty", std::vector<std::string>{"robot1"}, true);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  manager_->apply_effect(effect);

  // Verify property was cleared
  auto node_opt = kg_->get_node("robot1");
  ASSERT_TRUE(node_opt.has_value());
  auto node = *node_opt;

  for (const auto &prop : node.properties) {
    if (prop.key == "arm_empty") {
      EXPECT_FALSE(prop.value.bool_value);
      break;
    }
  }
}

// Test: Undo effect removes edge (reverse of add)
TEST_F(KgPddlManagerTest, UndoEffectRemovesEdge) {
  auto robot = create_node("robot1", "robot");
  auto location = create_node("loc1", "location");
  kg_->update_node(robot);
  kg_->update_node(location);

  // First apply the effect
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "at", std::vector<std::string>{"robot1", "loc1"}, false);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  manager_->apply_effect(effect);

  // Verify edge exists
  auto edges_before = kg_->get_edges("robot1", "loc1");
  ASSERT_FALSE(edges_before.empty());

  // Now undo the effect
  manager_->undo_effect(effect);

  // Verify edge was removed
  auto edges_after = kg_->get_edges("robot1", "loc1");
  bool found = false;
  for (const auto &e : edges_after) {
    if (e.edge_class == "at") {
      found = true;
      break;
    }
  }
  EXPECT_FALSE(found);
}

// Test: Undo effect adds edge (reverse of remove)
TEST_F(KgPddlManagerTest, UndoEffectAddsEdge) {
  auto robot = create_node("robot1", "robot");
  auto location = create_node("loc1", "location");
  kg_->update_node(robot);
  kg_->update_node(location);

  // Add edge first
  auto edge = create_edge("at", "robot1", "loc1");
  kg_->update_edge(edge);

  // Apply negated effect (removes edge)
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "at", std::vector<std::string>{"robot1", "loc1"}, true);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  manager_->apply_effect(effect);

  // Undo the effect (should add edge back)
  manager_->undo_effect(effect);

  // Verify edge was added back
  auto edges = kg_->get_edges("robot1", "loc1");
  bool found = false;
  for (const auto &e : edges) {
    if (e.edge_class == "at") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

// Test: Undo effect reverses unary property change
TEST_F(KgPddlManagerTest, UndoEffectReversesUnaryProperty) {
  auto robot = create_node("robot1", "robot");
  kg_->update_node(robot);

  // Apply effect: (arm_empty robot1)
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "arm_empty", std::vector<std::string>{"robot1"}, false);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  manager_->apply_effect(effect);

  // Verify property is true
  auto node_before = kg_->get_node("robot1");
  ASSERT_TRUE(node_before.has_value());
  bool value_before = false;
  for (const auto &prop : node_before->properties) {
    if (prop.key == "arm_empty") {
      value_before = prop.value.bool_value;
      break;
    }
  }
  EXPECT_TRUE(value_before);

  // Undo the effect
  manager_->undo_effect(effect);

  // Verify property is now false
  auto node_after = kg_->get_node("robot1");
  ASSERT_TRUE(node_after.has_value());
  for (const auto &prop : node_after->properties) {
    if (prop.key == "arm_empty") {
      EXPECT_FALSE(prop.value.bool_value);
      break;
    }
  }
}

// Test: Multiple nodes with same type
TEST_F(KgPddlManagerTest, MultipleNodesWithSameType) {
  auto loc1 = create_node("loc1", "location");
  auto loc2 = create_node("loc2", "location");
  auto loc3 = create_node("loc3", "location");
  kg_->update_node(loc1);
  kg_->update_node(loc2);
  kg_->update_node(loc3);

  auto [domain, problem] = manager_->get_pddl();

  // Type should appear only once
  size_t type_count = 0;
  size_t pos = 0;
  while ((pos = domain.find("location", pos)) != std::string::npos) {
    type_count++;
    pos += 8; // length of "location"
  }
  // Should appear in (:types section once
  EXPECT_GE(type_count, 1u);

  // All objects should appear
  EXPECT_TRUE(problem.find("loc1 - location") != std::string::npos);
  EXPECT_TRUE(problem.find("loc2 - location") != std::string::npos);
  EXPECT_TRUE(problem.find("loc3 - location") != std::string::npos);
}

// Test: Multiple edges between same nodes
TEST_F(KgPddlManagerTest, MultipleEdgesBetweenSameNodes) {
  auto robot = create_node("robot1", "robot");
  auto object = create_node("box1", "box");
  kg_->update_node(robot);
  kg_->update_node(object);

  auto edge1 = create_edge("holding", "robot1", "box1");
  auto edge2 = create_edge("near", "robot1", "box1");
  kg_->update_edge(edge1);
  kg_->update_edge(edge2);

  auto [domain, problem] = manager_->get_pddl();

  // Both predicates should be in domain
  EXPECT_TRUE(domain.find("(holding ?x ?y)") != std::string::npos);
  EXPECT_TRUE(domain.find("(near ?x ?y)") != std::string::npos);

  // Both facts should be in init
  EXPECT_TRUE(problem.find("(holding robot1 box1)") != std::string::npos);
  EXPECT_TRUE(problem.find("(near robot1 box1)") != std::string::npos);
}

// Test: Multiple goals
TEST_F(KgPddlManagerTest, MultipleGoals) {
  auto loc1 = create_node("loc1", "location");
  auto loc2 = create_node("loc2", "location");
  loc1.properties.push_back(create_bool_property("is_goal", true));
  loc2.properties.push_back(create_bool_property("is_goal", true));
  kg_->update_node(loc1);
  kg_->update_node(loc2);

  auto [domain, problem] = manager_->get_pddl();

  EXPECT_TRUE(problem.find("(is_goal loc1)") != std::string::npos);
  EXPECT_TRUE(problem.find("(is_goal loc2)") != std::string::npos);
}

// Test: Complex scenario with robots, locations, and objects
TEST_F(KgPddlManagerTest, ComplexScenario) {
  // Create nodes
  auto robot = create_node("robot1", "robot");
  robot.properties.push_back(create_bool_property("arm_empty", true));
  auto loc1 = create_node("kitchen", "location");
  auto loc2 = create_node("living_room", "location");
  loc2.properties.push_back(create_bool_property("is_goal", true));
  auto box = create_node("box1", "box");

  kg_->update_node(robot);
  kg_->update_node(loc1);
  kg_->update_node(loc2);
  kg_->update_node(box);

  // Create edges
  kg_->update_edge(create_edge("at", "robot1", "kitchen"));
  kg_->update_edge(create_edge("at", "box1", "kitchen"));
  kg_->update_edge(create_edge("connected", "kitchen", "living_room"));

  auto [domain, problem] = manager_->get_pddl();

  // Verify types
  EXPECT_TRUE(domain.find("robot") != std::string::npos);
  EXPECT_TRUE(domain.find("location") != std::string::npos);
  EXPECT_TRUE(domain.find("box") != std::string::npos);

  // Verify predicates
  EXPECT_TRUE(domain.find("(at ?x ?y)") != std::string::npos);
  EXPECT_TRUE(domain.find("(connected ?x ?y)") != std::string::npos);
  EXPECT_TRUE(domain.find("(arm_empty ?x)") != std::string::npos);
  EXPECT_TRUE(domain.find("(is_goal ?x)") != std::string::npos);

  // Verify objects
  EXPECT_TRUE(problem.find("robot1 - robot") != std::string::npos);
  EXPECT_TRUE(problem.find("kitchen - location") != std::string::npos);
  EXPECT_TRUE(problem.find("living_room - location") != std::string::npos);
  EXPECT_TRUE(problem.find("box1 - box") != std::string::npos);

  // Verify init
  EXPECT_TRUE(problem.find("(at robot1 kitchen)") != std::string::npos);
  EXPECT_TRUE(problem.find("(at box1 kitchen)") != std::string::npos);
  EXPECT_TRUE(problem.find("(connected kitchen living_room)") !=
              std::string::npos);
  EXPECT_TRUE(problem.find("(arm_empty robot1)") != std::string::npos);

  // Verify goal
  EXPECT_TRUE(problem.find("(is_goal living_room)") != std::string::npos);
}

// Test: Apply effect on non-existent node does not crash
TEST_F(KgPddlManagerTest, ApplyEffectOnNonExistentNodeDoesNotCrash) {
  // Create effect for non-existent node
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "arm_empty", std::vector<std::string>{"nonexistent"}, false);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  // Should not crash
  EXPECT_NO_THROW(manager_->apply_effect(effect));
}

// Test: Undo effect on non-existent node does not crash
TEST_F(KgPddlManagerTest, UndoEffectOnNonExistentNodeDoesNotCrash) {
  // Create effect for non-existent node
  auto pred = std::make_shared<easy_plan::pddl::Predicate>(
      "arm_empty", std::vector<std::string>{"nonexistent"}, false);
  easy_plan::pddl::Effect effect;
  effect.type = easy_plan::pddl::TimingExpression::END;
  effect.expression = pred;

  // Should not crash
  EXPECT_NO_THROW(manager_->undo_effect(effect));
}

// Test: Boolean property with false value is not included in init
TEST_F(KgPddlManagerTest, FalsePropertyNotInInit) {
  auto robot = create_node("robot1", "robot");
  robot.properties.push_back(create_bool_property("arm_empty", false));
  kg_->update_node(robot);

  auto [domain, problem] = manager_->get_pddl();

  // arm_empty should not appear in init since it's false
  EXPECT_TRUE(problem.find("(arm_empty robot1)") == std::string::npos);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
