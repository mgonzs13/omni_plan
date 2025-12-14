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

#include <knowledge_graph/graph_utils.hpp>
#include <knowledge_graph_msgs/msg/content.hpp>
#include <knowledge_graph_msgs/msg/graph_update.hpp>
#include <yasmin_ros/yasmin_node.hpp>

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

std::pair<std::string, std::string> KgPddlManager::get_pddl(
    std::vector<std::shared_ptr<easy_plan::pddl::Action>> actions_pddl) const {

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

  // Collect types from action parameters
  for (const auto &action : actions_pddl) {
    auto params = action->get_parameters();
    for (const auto &param : params) {
      types.insert(param.type);
    }
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

        if (source_node_opt->node_name == target_node_opt->node_name) {
          predicates.insert(edge.edge_class + " ?" + type_source[0] + "0 - " +
                            type_source);
        } else {
          predicates.insert(edge.edge_class + " ?" + type_source[0] + "0 - " +
                            type_source + " ?" + type_target[0] + "1 - " +
                            type_target);
        }
      }
    }
  }

  // Collect predicates from conditions and effects of actions
  for (const auto &action : actions_pddl) {

    auto params = action->get_parameters();

    for (const auto &cond : action->get_conditions()) {
      auto pred = cond.expression;
      auto args = pred->get_args();

      // Get types from action parameters
      std::string type1 = action->get_parameter_type(args[0]);

      if (args.size() == 1) {
        predicates.insert(pred->get_name() + " ?" + type1[0] + "0 - " + type1);
      } else if (args.size() == 2) {
        std::string type2 = action->get_parameter_type(args[1]);
        predicates.insert(pred->get_name() + " ?" + type1[0] + "0 - " + type1 +
                          " ?" + type2[0] + "1 - " + type2);
      }
    }

    for (const auto &eff : action->get_effects()) {
      auto pred = eff.expression;
      auto args = pred->get_args();

      // Get types from action parameters
      std::string type1 = action->get_parameter_type(args[0]);

      if (args.size() == 1) {
        predicates.insert(pred->get_name() + " ?" + type1[0] + "0 - " + type1);
      } else if (args.size() == 2) {
        std::string type2 = action->get_parameter_type(args[1]);
        predicates.insert(pred->get_name() + " ?" + type1[0] + "0 - " + type1 +
                          " ?" + type2[0] + "1 - " + type2);
      }
    }
  }

  // Define predicates
  if (!predicates.empty()) {
    domain += "(:predicates\n";
    for (const auto &pred : predicates) {
      domain += "  (" + pred + ")\n"; // Assuming binary predicates
    }
    domain += ")\n\n";
  }

  // Actions
  std::string action_str;
  for (const auto &action : actions_pddl) {
    action_str += action->to_pddl() + "\n";
  }

  domain += action_str + "\n";

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

void KgPddlManager::apply_effect(easy_plan::pddl::Effect exp) {
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

void KgPddlManager::undo_effect(easy_plan::pddl::Effect exp) {
  auto pred = exp.expression;
  bool is_negative = !pred->is_negated(); // Reverse for undo
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