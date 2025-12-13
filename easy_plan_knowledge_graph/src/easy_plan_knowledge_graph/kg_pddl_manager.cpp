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

#include <iostream>
#include <set>
#include <yasmin_ros/yasmin_node.hpp>

#include <knowledge_graph_msgs/msg/content.hpp>

#include "easy_plan_knowledge_graph/kg_pddl_manager.hpp"

using namespace easy_plan;
using namespace easy_plan_knowledge_graph;

KgPddlManager::KgPddlManager(
    std::shared_ptr<knowledge_graph::KnowledgeGraph> kg)
    : PddlManager(), kg_(kg ? kg
                            : std::make_shared<knowledge_graph::KnowledgeGraph>(
                                  yasmin_ros::YasminNode::get_instance())) {}

std::pair<std::string, std::string> KgPddlManager::get_pddl() const {

  auto nodes = this->kg_->get_nodes();
  auto edges = this->kg_->get_edges();

  // Build domain
  std::string domain = "(define (domain knowledge_graph_domain)\n\n";

  // Collect types
  std::set<std::string> types;
  for (const auto &node : nodes) {
    if (!node.node_class.empty()) {
      types.insert(node.node_class);
    }
  }

  if (!types.empty()) {
    domain += "(:types\n";
    for (const auto &type : types) {
      domain += "  " + type + "\n";
    }
    domain += ")\n\n";
  }

  // Collect predicates from edges
  std::set<std::string> predicates;
  for (const auto &edge : edges) {
    if (!edge.edge_class.empty()) {
      predicates.insert(edge.edge_class);
    }
  }

  // Collect unary predicates from boolean node properties
  std::set<std::string> unary_predicates;
  bool has_goals = false;
  for (const auto &node : nodes) {
    for (const auto &prop : node.properties) {
      if (prop.value.type == knowledge_graph_msgs::msg::Content::BOOL &&
          prop.value.bool_value) {
        if (prop.key == "is_goal") {
          has_goals = true;
        } else {
          unary_predicates.insert(prop.key);
        }
      }
    }
  }
  if (has_goals) {
    unary_predicates.insert("is_goal");
  }

  if (!predicates.empty() || !unary_predicates.empty()) {
    domain += "(:predicates\n";
    for (const auto &pred : predicates) {
      domain += "  (" + pred + " ?x ?y)\n"; // Assuming binary predicates
    }
    for (const auto &pred : unary_predicates) {
      domain += "  (" + pred + " ?x)\n"; // Unary predicates
    }
    domain += ")\n\n";
  }

  domain += ")";

  // Build problem
  std::string problem = "(define (problem knowledge_graph_problem)\n";
  problem += "(:domain knowledge_graph_domain)\n\n";

  // Objects
  if (!nodes.empty()) {
    problem += "(:objects\n";
    for (const auto &node : nodes) {
      problem += "  " + node.node_name + " - " + node.node_class + "\n";
    }
    problem += ")\n\n";
  }

  // Init
  problem += "(:init\n";

  // From edges
  for (const auto &edge : edges) {
    problem += "  (" + edge.edge_class + " " + edge.source_node + " " +
               edge.target_node + ")\n";
  }

  // Add unary predicates from boolean node properties
  for (const auto &node : nodes) {
    for (const auto &prop : node.properties) {
      if (prop.value.type == knowledge_graph_msgs::msg::Content::BOOL &&
          prop.value.bool_value && prop.key != "is_goal") {
        problem += "  (" + prop.key + " " + node.node_name + ")\n";
      }
    }
  }

  problem += ")\n\n";

  // Goal
  problem += "(:goal (and";
  for (const auto &node : nodes) {
    for (const auto &prop : node.properties) {
      if (prop.key == "is_goal" &&
          prop.value.type == knowledge_graph_msgs::msg::Content::BOOL &&
          prop.value.bool_value) {
        problem += " (is_goal " + node.node_name + ")";
      }
    }
  }
  problem += "))\n\n";

  problem += ")";

  return std::make_pair(domain, problem);
}

bool KgPddlManager::has_goals() const {
  auto nodes = this->kg_->get_nodes();
  for (const auto &node : nodes) {
    for (const auto &prop : node.properties) {
      if (prop.key == "is_goal" &&
          prop.value.type == knowledge_graph_msgs::msg::Content::BOOL &&
          prop.value.bool_value) {
        return true;
      }
    }
  }
  return false;
}

void KgPddlManager::apply_effect(easy_plan::pddl::Effect exp) {
  auto pred = exp.expression;
  bool is_negative = pred->is_negated();
  std::string name = pred->get_name();
  auto args = pred->get_args();

  if (args.size() == 1) {
    // Unary predicate: set boolean property on node
    std::string node_name = args[0];
    auto node_opt = kg_->get_node(node_name);
    if (node_opt) {
      auto node = *node_opt;
      bool found = false;
      for (auto &prop : node.properties) {
        if (prop.key == name) {
          prop.value.type = knowledge_graph_msgs::msg::Content::BOOL;
          prop.value.bool_value = !is_negative;
          found = true;
          break;
        }
      }
      if (!found) {
        knowledge_graph_msgs::msg::Property new_prop;
        new_prop.key = name;
        new_prop.value.type = knowledge_graph_msgs::msg::Content::BOOL;
        new_prop.value.bool_value = !is_negative;
        node.properties.push_back(new_prop);
      }
      kg_->update_node(node);
    }
  } else if (args.size() == 2) {
    // Binary predicate: add or remove edge
    std::string source = args[0];
    std::string target = args[1];
    if (!is_negative) {
      // Add edge
      knowledge_graph_msgs::msg::Edge edge;
      edge.edge_class = name;
      edge.source_node = source;
      edge.target_node = target;
      kg_->update_edge(edge);
    } else {
      // Remove edge
      auto edges = kg_->get_edges(source, target);
      for (const auto &e : edges) {
        if (e.edge_class == name) {
          kg_->remove_edge(e);
          break;
        }
      }
    }
  }
}

void KgPddlManager::undo_effect(easy_plan::pddl::Effect exp) {
  auto pred = exp.expression;
  bool is_negative = !pred->is_negated(); // Reverse for undo
  std::string name = pred->get_name();
  auto args = pred->get_args();

  if (args.size() == 1) {
    // Unary predicate: set boolean property on node
    std::string node_name = args[0];
    auto node_opt = kg_->get_node(node_name);
    if (node_opt) {
      auto node = *node_opt;
      bool found = false;
      for (auto &prop : node.properties) {
        if (prop.key == name) {
          prop.value.type = knowledge_graph_msgs::msg::Content::BOOL;
          prop.value.bool_value = !is_negative;
          found = true;
          break;
        }
      }
      if (!found) {
        knowledge_graph_msgs::msg::Property new_prop;
        new_prop.key = name;
        new_prop.value.type = knowledge_graph_msgs::msg::Content::BOOL;
        new_prop.value.bool_value = !is_negative;
        node.properties.push_back(new_prop);
      }
      kg_->update_node(node);
    }
  } else if (args.size() == 2) {
    // Binary predicate: add or remove edge
    std::string source = args[0];
    std::string target = args[1];
    if (!is_negative) {
      // Add edge
      knowledge_graph_msgs::msg::Edge edge;
      edge.edge_class = name;
      edge.source_node = source;
      edge.target_node = target;
      kg_->update_edge(edge);
    } else {
      // Remove edge
      auto edges = kg_->get_edges(source, target);
      for (const auto &e : edges) {
        if (e.edge_class == name) {
          kg_->remove_edge(e);
          break;
        }
      }
    }
  }
}