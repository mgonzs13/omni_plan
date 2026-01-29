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
    auto source_node = this->kg_->get_node(edge.get_source_node());
    auto target_node = this->kg_->get_node(edge.get_target_node());

    std::string name = edge.get_type();
    std::vector<std::string> args;

    if (source_node.get_name() == target_node.get_name()) {
      args.push_back(source_node.get_type());
    } else {
      args.push_back(source_node.get_type());
      args.push_back(target_node.get_type());
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

    auto source_node = this->kg_->get_node(edge.get_source_node());
    auto target_node = this->kg_->get_node(edge.get_target_node());
    std::string name = edge.get_type();

    std::vector<std::string> args;

    if (source_node.get_name() == target_node.get_name()) {
      args.push_back(source_node.get_name());
    } else {
      args.push_back(source_node.get_name());
      args.push_back(target_node.get_name());
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

  auto edges = this->kg_->get_edges();
  for (const auto &edge : edges) {
    if (!edge.has_property("is_goal")) {
      continue;
    }

    if (edge.get_property<bool>("is_goal")) {
      return true;
    }
  }

  std::unique_lock<std::mutex> lock(this->goal_mutex_);
  this->goal_cv_.wait(lock);

  return false;
}

bool KgPddlManager::predicate_exists(
    const omni_plan::pddl::Predicate &predicate) const {

  std::string name = predicate.get_name();
  auto args = predicate.get_args();

  std::string source = args[0];
  std::string target = args.size() == 2 ? args[1] : args[0];

  return this->kg_->has_edge(name, source, target);
}

bool KgPddlManager::predicate_is_goal(
    const omni_plan::pddl::Predicate &predicate) const {

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

void KgPddlManager::apply_effect(const omni_plan::pddl::Effect &exp) {
  auto pred = exp;
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
    if (edge.has_property("is_goal") && edge.get_property<bool>("is_goal")) {
      this->goal_cv_.notify_all();
      break;
    }
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(KgPddlManager, omni_plan::PddlManager)