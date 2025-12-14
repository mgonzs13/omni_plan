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
#include <mutex>
#include <set>

#include "knowledge_graph/graph_utils.hpp"
#include "knowledge_graph_msgs/msg/content.hpp"
#include "knowledge_graph_msgs/msg/graph_update.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "easy_plan_knowledge_graph/kg_pddl_manager.hpp"

using namespace easy_plan;
using namespace easy_plan_knowledge_graph;

KgPddlManager::KgPddlManager(
    std::shared_ptr<knowledge_graph::KnowledgeGraph> kg)
    : PddlManager(), kg_(kg ? kg
                            : std::make_shared<knowledge_graph::KnowledgeGraph>(
                                  yasmin_ros::YasminNode::get_instance())) {
  auto node = yasmin_ros::YasminNode::get_instance();
  this->update_sub_ =
      node->create_subscription<knowledge_graph_msgs::msg::GraphUpdate>(
          "graph_update", rclcpp::QoS(100).reliable(),
          std::bind(&KgPddlManager::graph_update_callback, this,
                    std::placeholders::_1));
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
    if (!node.node_class.empty()) {
      types.insert(node.node_class);
    }
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
    if (!edge.edge_class.empty()) {
      auto source_node_opt = this->kg_->get_node(edge.source_node);
      auto target_node_opt = this->kg_->get_node(edge.target_node);
      if (source_node_opt && target_node_opt) {
        std::string type_source = source_node_opt->node_class;
        std::string type_target = target_node_opt->node_class;
        std::string predicate = "(" + edge.edge_class;

        if (source_node_opt->node_name == target_node_opt->node_name) {
          predicate +=
              " ?" + std::string(1, type_source[0]) + "0 - " + type_source;
        } else {
          predicate += " ?" + std::string(1, type_source[0]) + "0 - " +
                       type_source + " ?" + std::string(1, type_target[0]) +
                       "1 - " + type_target;
        }
        predicate += ")";
        predicates.insert(predicate);
      }
    }
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
      problem += "  " + node.node_name + " - " + node.node_class + "\n";
    }
    problem += ")\n\n";
  }

  // Init
  problem += "(:init\n";

  // From edges
  for (const auto &edge : edges) {
    auto is_goal = knowledge_graph::get_property<bool>(edge, "is_goal");

    if (is_goal.has_value() && is_goal.value()) {
      continue; // Skip goal edges
    }

    if (edge.source_node == edge.target_node) {
      problem += "  (" + edge.edge_class + " " + edge.source_node + ")\n";

    } else {
      problem += "  (" + edge.edge_class + " " + edge.source_node + " " +
                 edge.target_node + ")\n";
    }
  }

  problem += ")\n\n";

  // Goal
  problem += "(:goal (and";

  // From edges marked as goals
  for (const auto &edge : edges) {
    for (const auto &prop : edge.properties) {
      if (prop.key == "is_goal" &&
          prop.value.type == knowledge_graph_msgs::msg::Content::BOOL &&
          prop.value.bool_value) {
        problem += " ( " + edge.edge_class + " " + edge.source_node + " " +
                   edge.target_node + ")";
        ;
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
    auto is_goal = knowledge_graph::get_property<bool>(edge, "is_goal");

    if (is_goal.has_value() && is_goal.value()) {
      return true;
    }
  }
  return false;
}

bool KgPddlManager::predicate_exists(
    const easy_plan::pddl::Predicate &predicate) const {

  std::string name = predicate.get_name();
  auto args = predicate.get_args();

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  auto edges = this->kg_->get_edges(source, target);
  for (const auto &e : edges) {
    if (e.edge_class == name) {
      return true;
    }
  }

  return false;
}

bool KgPddlManager::predicate_is_goal(
    const easy_plan::pddl::Predicate &predicate) const {

  std::string name = predicate.get_name();
  auto args = predicate.get_args();

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  auto edges = this->kg_->get_edges(source, target);
  for (const auto &e : edges) {
    if (e.edge_class == name) {
      auto is_goal = knowledge_graph::get_property<bool>(e, "is_goal");
      if (is_goal.has_value() && is_goal.value()) {
        return true;
      }
    }
  }

  return false;
}

void KgPddlManager::apply_effect(const easy_plan::pddl::Effect &exp) {
  auto pred = exp.expression;
  bool is_negative = pred->is_negated();
  std::string name = pred->get_name();
  auto args = pred->get_args();

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  if (!is_negative) {
    // Add edge
    knowledge_graph_msgs::msg::Edge edge;
    edge.edge_class = name;
    edge.source_node = source;
    edge.target_node = target;
    this->kg_->update_edge(edge);
  } else {
    // Remove edge
    auto edges = this->kg_->get_edges(source, target);
    for (const auto &e : edges) {
      if (e.edge_class == name) {
        this->kg_->remove_edge(e);
        break;
      }
    }
  }
}

void KgPddlManager::graph_update_callback(
    const knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg) {
  (void)msg;
  this->goal_cv_.notify_all();
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(KgPddlManager, easy_plan::PddlManager)