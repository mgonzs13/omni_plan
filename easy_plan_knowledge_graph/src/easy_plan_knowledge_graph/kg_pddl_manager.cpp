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

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <set>

#include "knowledge_graph/graph/edge.hpp"

#include "easy_plan_knowledge_graph/kg_pddl_manager.hpp"

using namespace easy_plan;
using namespace easy_plan_knowledge_graph;

KgPddlManager::KgPddlManager()
    : PddlManager(), kg_(knowledge_graph::KnowledgeGraph::get_instance()) {
  this->kg_->add_callback(
      std::bind(&KgPddlManager::graph_callback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
}

std::pair<std::string, std::string>
KgPddlManager::get_pddl(std::set<std::string> actions_types,
                        std::set<std::string> actions_predicates,
                        std::set<std::string> actions_pddl) const {

  auto nodes = this->kg_->get_nodes();
  auto edges = this->kg_->get_edges();

  // Build domain
  std::string domain = "(define (domain knowledge_graph_domain)\n\n";
  domain +=
      "(:requirements :typing :durative-actions :negative-preconditions)\n\n ";

  // Collect types
  std::set<std::string> types;
  for (const auto &node : nodes) {
    types.insert(node.get_type());
  }

  // Add action types
  for (const auto &type : actions_types) {
    types.insert(type);
  }

  // Define types
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
    auto source_node = this->kg_->get_node(edge.get_source_node());
    auto target_node = this->kg_->get_node(edge.get_target_node());
    std::string type_source = source_node.get_type();
    std::string type_target = target_node.get_type();
    std::string predicate = "(" + edge.get_type();

    if (source_node.get_name() == target_node.get_name()) {
      predicate += " ?" + std::string(1, type_source[0]) + "0 - " + type_source;
    } else {
      predicate += " ?" + std::string(1, type_source[0]) + "0 - " +
                   type_source + " ?" + std::string(1, type_target[0]) +
                   "1 - " + type_target;
    }
    predicate += ")";
    predicates.insert(predicate);
  }

  // Collect predicates from conditions and effects of actions
  for (const auto &pred_str : actions_predicates) {
    predicates.insert(pred_str);
  }

  // Define predicates
  if (!predicates.empty()) {
    domain += "(:predicates\n";
    for (const auto &pred : predicates) {
      domain += "  " + pred + "\n"; // Assuming binary predicates
    }
    domain += ")\n\n";
  }

  // Actions
  for (const auto &action_str : actions_pddl) {
    domain += action_str + "\n";
  }

  domain += ")";

  // Build problem
  std::string problem = "(define (problem knowledge_graph_problem)\n";
  problem += "(:domain knowledge_graph_domain)\n\n";

  // Objects
  if (!nodes.empty()) {
    problem += "(:objects\n";
    for (const auto &node : nodes) {
      problem += "  " + node.get_name() + " - " + node.get_type() + "\n";
    }
    problem += ")\n\n";
  }

  // Init
  problem += "(:init\n";

  // From edges
  for (const auto &edge : edges) {
    if (edge.has_property("is_goal")) {
      if (edge.get_property<bool>("is_goal")) {
        continue;
      }
    }

    if (edge.get_source_node() == edge.get_target_node()) {
      problem += "  (" + edge.get_type() + " " + edge.get_source_node() + ")\n";

    } else {
      problem += "  (" + edge.get_type() + " " + edge.get_source_node() + " " +
                 edge.get_target_node() + ")\n";
    }
  }

  problem += ")\n\n";

  // Goal
  problem += "(:goal (and";

  // From edges marked as goals
  for (const auto &edge : edges) {
    if (edge.has_property("is_goal")) {
      if (edge.get_property<bool>("is_goal")) {
        if (edge.get_source_node() == edge.get_target_node()) {
          problem +=
              " ( " + edge.get_type() + " " + edge.get_source_node() + " )";
        } else {
          problem += " ( " + edge.get_type() + " " + edge.get_source_node() +
                     " " + edge.get_target_node() + ")";
        }
      }
    }
  }

  problem += "))\n\n";

  problem += ")";

  return std::make_pair(domain, problem);
}

bool KgPddlManager::has_goals() const {

  std::unique_lock<std::mutex> lock(this->goal_mutex_);
  this->goal_cv_.wait(lock);

  auto edges = this->kg_->get_edges();
  for (const auto &edge : edges) {
    if (!edge.has_property("is_goal")) {
      continue;
    }

    return true;
  }
  return false;
}

bool KgPddlManager::predicate_exists(
    const easy_plan::pddl::Predicate &predicate) const {

  std::string name = predicate.get_name();
  auto args = predicate.get_args();

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  return this->kg_->has_edge(name, source, target);
}

bool KgPddlManager::predicate_is_goal(
    const easy_plan::pddl::Predicate &predicate) const {

  std::string name = predicate.get_name();
  auto args = predicate.get_args();

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  if (!this->kg_->has_edge(name, source, target)) {
    return false;
  }

  auto edge = this->kg_->get_edge(name, source, target);

  if (edge.has_property("is_goal")) {
    return edge.get_property<bool>("is_goal");
  }

  return false;
}

void KgPddlManager::apply_effect(const easy_plan::pddl::Effect &exp) {
  auto pred = exp.expression;
  bool is_negative = pred.is_negated();
  std::string name = pred.get_name();
  auto args = pred.get_args();

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  knowledge_graph::graph::Edge edge(name, source, target);

  if (!is_negative) {
    // Add edge
    this->kg_->update_edge(edge);
  } else {
    // Remove edge
    this->kg_->remove_edge(edge);
  }
}

void KgPddlManager::graph_callback(
    const std::string &operation, const std::string &element_type,
    const std::vector<std::variant<knowledge_graph::graph::Node,
                                   knowledge_graph::graph::Edge>> &elements) {
  if (element_type != "edge" && (operation != "add" || operation != "update")) {
    return;
  }

  for (const auto &elem : elements) {
    const auto &edge = std::get<knowledge_graph::graph::Edge>(elem);
    if (edge.has_property("is_goal")) {
      if (edge.get_property<bool>("is_goal")) {
        this->goal_cv_.notify_one();
        break;
      }
    }
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(KgPddlManager, easy_plan::PddlManager)