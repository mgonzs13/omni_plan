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

#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <set>
#include <vector>

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
    : PddlManager(), kg_(knowledge_graph::KnowledgeGraph::get_instance()),
      callback_state_(std::make_shared<CallbackState>()) {
  if (!add_callback) {
    return;
  }

  auto state = this->callback_state_;
  this->kg_->add_callback(
      [this,
       state](const std::string &operation, const std::string &element_type,
              const std::vector<std::variant<knowledge_graph::graph::Node,
                                             knowledge_graph::graph::Edge>>
                  &elements) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->alive.load()) {
          return;
        }
        this->graph_callback(operation, element_type, elements);
      });
}

KgPddlManager::~KgPddlManager() {
  if (this->callback_state_) {
    std::lock_guard<std::mutex> lock(this->callback_state_->mutex);
    this->callback_state_->alive.store(false);
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

  // Collect predicates from edges, deduplicating by predicate signature
  std::map<std::string, std::vector<std::string>> predicate_signatures;
  std::set<std::string> reported_conflicts;

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

    auto signature = predicate_signatures.find(name);

    if (signature == predicate_signatures.end()) {
      predicate_signatures[name] = args;
      domain.add_predicate(omni_plan::pddl::Predicate(name, args));
    } else if (signature->second != args &&
               reported_conflicts.insert(name).second) {
      RCLCPP_WARN(rclcpp::get_logger("kg_pddl_manager"),
                  "Ignoring predicate '%s' with conflicting arguments",
                  name.c_str());
    }
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

bool KgPddlManager::scan_goals() const {

  for (const auto &edge : this->kg_->get_edges()) {
    if (edge.has_property("is_goal") && edge.get_property<bool>("is_goal")) {
      return true;
    }
  }

  return false;
}

bool KgPddlManager::has_goals() const {

  if (!this->has_goals_.load()) {
    std::unique_lock<std::mutex> lock(this->goal_mutex_);
    this->goal_cv_.wait_for(lock, std::chrono::milliseconds(100),
                            [this] { return this->has_goals_.load(); });
  }

  // The cached flag is only a hint used to skip the wait above: it can be
  // stale because fulfilling a goal flips is_goal to false through
  // update_edge, whose callback carries no goal edge and never clears the
  // flag. Always confirm against the graph here; otherwise a stale true makes
  // the state machine loop forever on goal-less problems (an idle storm).
  const bool goal_found = this->scan_goals();
  this->has_goals_.store(goal_found);
  return goal_found;
}

bool KgPddlManager::clear_goals() const {

  bool success = true;

  for (const auto &edge : this->kg_->get_edges()) {
    if (!edge.has_property("is_goal")) {
      continue;
    }

    if (edge.get_property<bool>("is_goal") && !this->kg_->remove_edge(edge)) {
      success = false;
    }
  }

  if (success) {
    this->has_goals_.store(false);
  }

  return success;
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

  if (!is_negative) {
    // Add edge, preserving any existing edge properties
    knowledge_graph::graph::Edge edge(name, source, target);

    try {
      edge = this->kg_->get_edge(name, source, target);
    } catch (const std::runtime_error &) {
    }

    edge.set_property("is_goal", false);
    this->kg_->update_edge(edge);
  } else {
    // Remove edge
    knowledge_graph::graph::Edge edge(name, source, target);
    this->kg_->remove_edge(edge);
  }
}

void KgPddlManager::graph_callback(
    const std::string &operation, const std::string &element_type,
    const std::vector<std::variant<knowledge_graph::graph::Node,
                                   knowledge_graph::graph::Edge>> &elements) {

  if (element_type != "edge") {
    return;
  }

  bool goal_edge = false;
  for (const auto &elem : elements) {
    const auto *edge = std::get_if<knowledge_graph::graph::Edge>(&elem);
    if (edge != nullptr && edge->has_property("is_goal") &&
        edge->get_property<bool>("is_goal")) {
      goal_edge = true;
      break;
    }
  }

  if (!goal_edge) {
    return;
  }

  if (operation == "add" || operation == "update") {
    this->has_goals_.store(true);
  } else if (operation == "remove") {
    this->has_goals_.store(false);
  } else {
    return;
  }

  std::lock_guard<std::mutex> lock(this->goal_mutex_);
  this->goal_cv_.notify_all();
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(KgPddlManager, omni_plan::PddlManager)