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
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/pddl_manager.hpp"

#include "omni_plan_knowledge_base/kb_pddl_manager.hpp"

using namespace omni_plan;
using namespace omni_plan_knowledge_base;

KbPddlManager::KbPddlManager() : PddlManager() {
  // Create knowledge base client
  this->kb_client_ = std::make_shared<KnowledgeBaseClient>("kb_pddl_manager");

  // Register callback for knowledge updates
  this->kb_client_->add_knowledge_update_callback(std::bind(
      &KbPddlManager::knowledge_update_callback, this, std::placeholders::_1));
}

KbPddlManager::~KbPddlManager() {
  // KnowledgeBaseClient destructor will handle cleanup
}

std::pair<omni_plan::pddl::Domain, omni_plan::pddl::Problem>
KbPddlManager::get_pddl() const {

  omni_plan::pddl::Domain domain;
  omni_plan::pddl::Problem problem;

  // Get types
  auto types = this->kb_client_->get_types();
  for (const auto &type : types) {
    domain.add_type(type);
  }

  // Get predicates
  auto predicates = this->kb_client_->get_predicates();
  for (const auto &predicate : predicates) {
    domain.add_predicate(predicate);
  }

  // Get objects
  auto objects = this->kb_client_->get_objects();
  for (const auto &object : objects) {
    problem.add_object(object);
  }

  // Get facts
  auto facts = this->kb_client_->get_facts();
  for (const auto &fact : facts) {
    problem.add_fact(fact);
  }

  // Get goals
  auto goals = this->kb_client_->get_goals();
  for (const auto &goal : goals) {
    problem.add_goal(goal);
  }

  return std::make_pair(domain, problem);
}

bool KbPddlManager::has_goals() const {
  if (this->kb_client_->has_goals()) {
    return true;
  }

  // Wait for goals to be added
  std::unique_lock<std::mutex> lock(this->goal_mutex_);
  this->goal_cv_.wait(lock);

  return false;
}

bool KbPddlManager::predicate_exists(
    const omni_plan::pddl::Predicate &predicate) const {

  auto facts = this->kb_client_->get_facts(predicate.get_name());
  auto pred_args = predicate.get_args();

  for (const auto &fact : facts) {
    if (fact.get_name() == predicate.get_name()) {
      auto fact_args = fact.get_args();
      if (fact_args.size() == pred_args.size()) {
        bool match = true;
        for (size_t i = 0; i < pred_args.size(); ++i) {
          if (fact_args[i] != pred_args[i]) {
            match = false;
            break;
          }
        }
        if (match) {
          return true;
        }
      }
    }
  }

  return false;
}

bool KbPddlManager::predicate_is_goal(
    const omni_plan::pddl::Predicate &predicate) const {

  auto goals = this->kb_client_->get_goals();
  auto pred_args = predicate.get_args();

  for (const auto &goal : goals) {
    if (goal.get_name() == predicate.get_name()) {
      auto goal_args = goal.get_args();
      if (goal_args.size() == pred_args.size()) {
        bool match = true;
        for (size_t i = 0; i < pred_args.size(); ++i) {
          if (goal_args[i] != pred_args[i]) {
            match = false;
            break;
          }
        }
        if (match) {
          return true;
        }
      }
    }
  }

  return false;
}

void KbPddlManager::apply_effect(const omni_plan::pddl::Effect &exp) {
  auto pred = exp;
  bool is_negative = pred.is_negated();

  if (!is_negative) {
    // Add fact
    this->kb_client_->add_fact(pred);
  } else {
    // Remove fact
    omni_plan::pddl::Predicate fact_to_remove(pred.get_name(), pred.get_args(),
                                              false);
    this->kb_client_->remove_fact(fact_to_remove);
  }
}

void KbPddlManager::knowledge_update_callback(
    const omni_plan_msgs::msg::KnowledgeUpdate::SharedPtr msg) {

  // If a goal was added, notify waiting threads
  if (msg->entity_type == omni_plan_msgs::msg::KnowledgeUpdate::GOAL &&
      msg->operation == omni_plan_msgs::msg::KnowledgeUpdate::ADD) {
    this->goal_cv_.notify_all();
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(KbPddlManager, omni_plan::PddlManager)
