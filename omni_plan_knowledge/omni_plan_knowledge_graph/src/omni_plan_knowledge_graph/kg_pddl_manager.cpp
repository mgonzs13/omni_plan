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

#include "rclcpp/rclcpp.hpp"

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/pddl_manager.hpp"

#include "omni_plan_knowledge_graph/kg_pddl_manager.hpp"

using namespace omni_plan;
using namespace omni_plan_knowledge_graph;

KgPddlManager::KgPddlManager(bool add_callback)
    : PddlManager(), kg_(knowledge_graph::KnowledgeGraph::get_instance()) {
  if (add_callback) {
    this->kg_->add_callback(
        std::bind(&KgPddlManager::graph_callback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
  }
}

inline std::optional<knowledge_graph::graph::Node>
search_node(const std::vector<knowledge_graph::graph::Node> &nodes,
            std::string source_node_name) {

  auto it = std::find_if(
      nodes.begin(), nodes.end(),
      [&source_node_name](const knowledge_graph::graph::Node &node) {
        return node.get_name() == source_node_name;
      });

  if (it != nodes.end()) {
    return *it;
  }

  return std::nullopt;
}

std::pair<omni_plan::pddl::Domain, omni_plan::pddl::Problem>
KgPddlManager::get_pddl() const {

  auto nodes = this->kg_->get_nodes();
  auto edges = this->kg_->get_edges();

  omni_plan::pddl::Domain domain;
  omni_plan::pddl::Problem problem;

  // Collect types
  for (const auto &node : nodes) {
    domain.add_type(node.get_type());
  }

  // Collect predicates from edges
  std::set<std::string> predicates;

  for (const auto &edge : edges) {
    std::string source_node_name = edge.get_source_node();
    std::string target_node_name = edge.get_target_node();

    // Find the source and target nodes in the nodes list
    auto source_node = search_node(nodes, source_node_name);
    auto target_node = search_node(nodes, target_node_name);

    if (!source_node || !target_node) {
      continue;
    }

    std::string name = edge.get_type();
    std::vector<std::string> args;

    if (source_node->get_name() == target_node->get_name()) {
      args.push_back(source_node->get_type());
    } else {
      args.push_back(source_node->get_type());
      args.push_back(target_node->get_type());
    }

    domain.add_predicate(omni_plan::pddl::Predicate(name, args));
  }

  // Objects
  for (const auto &node : nodes) {
    problem.add_object(
        omni_plan::pddl::Object(node.get_name(), node.get_type()));
  }

  // From edges
  for (const auto &edge : edges) {

    std::string source_node_name = edge.get_source_node();
    std::string target_node_name = edge.get_target_node();

    // Find the source and target nodes in the nodes list
    auto source_node = search_node(nodes, source_node_name);
    auto target_node = search_node(nodes, target_node_name);

    if (!source_node || !target_node) {
      continue;
    }

    std::string name = edge.get_type();
    std::vector<std::string> args;

    if (source_node->get_name() == target_node->get_name()) {
      args.push_back(source_node->get_name());
    } else {
      args.push_back(source_node->get_name());
      args.push_back(target_node->get_name());
    }

    omni_plan::pddl::Predicate pred(name, args);

    if (edge.has_property("is_goal") && edge.get_property<bool>("is_goal")) {
      problem.add_goal(pred);
    } else {
      problem.add_fact(pred);
    }
  }

  return std::make_pair(domain, problem);
}

bool KgPddlManager::has_goals() const {

  auto has_goal_edge = [this]() {
    auto edges = this->kg_->get_edges();
    for (const auto &edge : edges) {
      if (edge.has_property("is_goal") && edge.get_property<bool>("is_goal")) {
        return true;
      }
    }
    return false;
  };

  if (!has_goal_edge()) {
    std::unique_lock<std::mutex> lock(this->goal_mutex_);
    this->goal_cv_.wait(lock, [&has_goal_edge] { return has_goal_edge(); });
  }

  return has_goal_edge();
}

bool KgPddlManager::clear_goals() const {

  auto edges = this->kg_->get_edges();
  for (const auto &edge : edges) {
    if (!edge.has_property("is_goal")) {
      continue;
    }

    if (edge.get_property<bool>("is_goal")) {
      this->kg_->remove_edge(edge);
    }
  }

  return true;
}

bool KgPddlManager::predicate_exists(
    const omni_plan::pddl::Predicate &predicate) const {

  std::string name = predicate.get_name();
  auto args = predicate.get_args();

  if (args.empty()) {
    return false;
  }

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  bool edge = false;
  try {
    auto edge_obj = this->kg_->get_edge(name, source, target);
    if (edge_obj.has_property("is_goal") &&
        edge_obj.get_property<bool>("is_goal")) {
      edge = false;
    } else {
      edge = true;
    }
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger("kg_pddl_manager"),
                 "Exception in predicate_exists: %s", e.what());
    edge = false;
  }

  return edge;
}

bool KgPddlManager::predicate_is_goal(
    const omni_plan::pddl::Predicate &predicate) const {

  std::string name = predicate.get_name();
  auto args = predicate.get_args();

  if (args.empty()) {
    return false;
  }

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  if (!this->kg_->has_edge(name, source, target)) {
    return false;
  }

  try {
    auto edge = this->kg_->get_edge(name, source, target);
    if (edge.has_property("is_goal")) {
      return edge.get_property<bool>("is_goal");
    }

  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger("kg_pddl_manager"),
                 "Exception in predicate_is_goal: %s", e.what());
    return false;
  }

  return false;
}

void KgPddlManager::apply_effect(const omni_plan::pddl::Effect &exp) {
  auto pred = exp;
  bool is_negative = pred.is_negated();
  std::string name = pred.get_name();
  auto args = pred.get_args();

  if (args.empty()) {
    return;
  }

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  knowledge_graph::graph::Edge edge(name, source, target);
  edge.set_property("is_goal", false);

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

  if (element_type != "edge" || (operation != "add" && operation != "update")) {
    return;
  }

  for (const auto &elem : elements) {
    const auto &edge = std::get<knowledge_graph::graph::Edge>(elem);
    if (edge.has_property("is_goal") && edge.get_property<bool>("is_goal")) {
      std::lock_guard<std::mutex> lock(this->goal_mutex_);
      this->goal_cv_.notify_all();
      break;
    }
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(KgPddlManager, omni_plan::PddlManager)