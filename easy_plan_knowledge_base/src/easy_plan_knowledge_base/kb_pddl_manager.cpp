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

#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/pddl_manager.hpp"

#include "easy_plan_knowledge_base/kb_pddl_manager.hpp"

using namespace easy_plan;
using namespace easy_plan_knowledge_base;
using namespace std::chrono_literals;

KbPddlManager::KbPddlManager() : PddlManager() {
  // Create a node for service clients
  this->node_ = rclcpp::Node::make_shared("kb_pddl_manager");

  // Create service clients
  this->get_types_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetTypes>("get_types");
  this->get_objects_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetObjects>(
          "get_objects");
  this->get_predicates_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetPredicates>(
          "get_predicates");
  this->get_facts_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetFacts>("get_facts");
  this->get_goals_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetGoals>("get_goals");
  this->add_fact_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddFact>("add_fact");
  this->remove_fact_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveFact>(
          "remove_fact");

  // Subscribe to knowledge updates if requested
  this->knowledge_update_sub_ =
      this->node_->create_subscription<easy_plan_msgs::msg::KnowledgeUpdate>(
          "knowledge_updates", 10,
          std::bind(&KbPddlManager::knowledge_update_callback, this,
                    std::placeholders::_1));

  // Create executor and start spinning in a separate thread
  this->executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor_->add_node(this->node_);
  this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });
}

KbPddlManager::~KbPddlManager() {
  if (this->executor_) {
    this->executor_->cancel();
  }
  if (this->executor_thread_.joinable()) {
    this->executor_thread_.join();
  }
}

std::pair<easy_plan::pddl::Domain, easy_plan::pddl::Problem>
KbPddlManager::get_pddl() const {

  easy_plan::pddl::Domain domain;
  easy_plan::pddl::Problem problem;

  // Get types
  auto types_request =
      std::make_shared<easy_plan_msgs::srv::GetTypes::Request>();
  if (!this->get_types_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(), "get_types service not available");
    return std::make_pair(domain, problem);
  }

  auto types_future =
      this->get_types_client_->async_send_request(types_request);
  if (types_future.wait_for(5s) == std::future_status::ready) {
    auto types_response = types_future.get();
    for (const auto &type : types_response->types) {
      domain.add_type(type);
    }
  }

  // Get predicates
  auto predicates_request =
      std::make_shared<easy_plan_msgs::srv::GetPredicates::Request>();
  if (!this->get_predicates_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "get_predicates service not available");
    return std::make_pair(domain, problem);
  }

  auto predicates_future =
      this->get_predicates_client_->async_send_request(predicates_request);
  if (predicates_future.wait_for(5s) == std::future_status::ready) {
    auto predicates_response = predicates_future.get();
    for (const auto &pred_msg : predicates_response->predicates) {
      std::vector<std::string> args;
      for (const auto &arg : pred_msg.arguments) {
        args.push_back(arg);
      }
      domain.add_predicate(easy_plan::pddl::Predicate(pred_msg.name, args));
    }
  }

  // Get objects
  auto objects_request =
      std::make_shared<easy_plan_msgs::srv::GetObjects::Request>();
  if (!this->get_objects_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "get_objects service not available");
    return std::make_pair(domain, problem);
  }

  auto objects_future =
      this->get_objects_client_->async_send_request(objects_request);
  if (objects_future.wait_for(5s) == std::future_status::ready) {
    auto objects_response = objects_future.get();
    for (const auto &obj_msg : objects_response->objects) {
      problem.add_object(easy_plan::pddl::Object(obj_msg.name, obj_msg.type));
    }
  }

  // Get facts
  auto facts_request =
      std::make_shared<easy_plan_msgs::srv::GetFacts::Request>();
  facts_request->name = ""; // Get all facts
  if (!this->get_facts_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(), "get_facts service not available");
    return std::make_pair(domain, problem);
  }

  auto facts_future =
      this->get_facts_client_->async_send_request(facts_request);
  if (facts_future.wait_for(5s) == std::future_status::ready) {
    auto facts_response = facts_future.get();
    for (const auto &fact_msg : facts_response->facts) {
      std::vector<std::string> args;
      for (const auto &arg : fact_msg.arguments) {
        args.push_back(arg);
      }
      problem.add_fact(easy_plan::pddl::Predicate(fact_msg.name, args));
    }
  }

  // Get goals
  auto goals_request =
      std::make_shared<easy_plan_msgs::srv::GetGoals::Request>();
  if (!this->get_goals_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(), "get_goals service not available");
    return std::make_pair(domain, problem);
  }

  auto goals_future =
      this->get_goals_client_->async_send_request(goals_request);
  if (goals_future.wait_for(5s) == std::future_status::ready) {
    auto goals_response = goals_future.get();
    for (const auto &goal_msg : goals_response->goals) {
      std::vector<std::string> args;
      for (const auto &arg : goal_msg.arguments) {
        args.push_back(arg);
      }
      problem.add_goal(easy_plan::pddl::Predicate(goal_msg.name, args));
    }
  }

  return std::make_pair(domain, problem);
}

bool KbPddlManager::has_goals() const {
  auto goals_request =
      std::make_shared<easy_plan_msgs::srv::GetGoals::Request>();

  if (!this->get_goals_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "get_goals service not available in has_goals");
    return false;
  }

  auto goals_future =
      this->get_goals_client_->async_send_request(goals_request);
  if (goals_future.wait_for(5s) == std::future_status::ready) {
    auto goals_response = goals_future.get();
    if (!goals_response->goals.empty()) {
      return true;
    }
  }

  // Wait for goals to be added
  std::unique_lock<std::mutex> lock(this->goal_mutex_);
  this->goal_cv_.wait(lock);

  return false;
}

bool KbPddlManager::predicate_exists(
    const easy_plan::pddl::Predicate &predicate) const {

  auto facts_request =
      std::make_shared<easy_plan_msgs::srv::GetFacts::Request>();
  facts_request->name = predicate.get_name();

  if (!this->get_facts_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "get_facts service not available in predicate_exists");
    return false;
  }

  auto facts_future =
      this->get_facts_client_->async_send_request(facts_request);
  if (facts_future.wait_for(5s) == std::future_status::ready) {
    auto facts_response = facts_future.get();

    auto pred_args = predicate.get_args();

    for (const auto &fact_msg : facts_response->facts) {
      if (fact_msg.name == predicate.get_name()) {
        // Check if arguments match
        if (fact_msg.arguments.size() == pred_args.size()) {
          bool match = true;
          for (size_t i = 0; i < pred_args.size(); ++i) {
            if (fact_msg.arguments[i] != pred_args[i]) {
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
  }

  return false;
}

bool KbPddlManager::predicate_is_goal(
    const easy_plan::pddl::Predicate &predicate) const {

  auto goals_request =
      std::make_shared<easy_plan_msgs::srv::GetGoals::Request>();

  if (!this->get_goals_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "get_goals service not available in predicate_is_goal");
    return false;
  }

  auto goals_future =
      this->get_goals_client_->async_send_request(goals_request);
  if (goals_future.wait_for(5s) == std::future_status::ready) {
    auto goals_response = goals_future.get();

    auto pred_args = predicate.get_args();

    for (const auto &goal_msg : goals_response->goals) {
      if (goal_msg.name == predicate.get_name()) {
        // Check if arguments match
        if (goal_msg.arguments.size() == pred_args.size()) {
          bool match = true;
          for (size_t i = 0; i < pred_args.size(); ++i) {
            if (goal_msg.arguments[i] != pred_args[i]) {
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
  }

  return false;
}

void KbPddlManager::apply_effect(const easy_plan::pddl::Effect &exp) {
  auto pred = exp;
  bool is_negative = pred.is_negated();

  easy_plan_msgs::msg::Predicate pred_msg;
  pred_msg.name = pred.get_name();
  pred_msg.arguments = pred.get_args();
  pred_msg.time = easy_plan_msgs::msg::Predicate::AT_END;
  pred_msg.negated = false;

  if (!is_negative) {
    // Add fact
    auto add_request =
        std::make_shared<easy_plan_msgs::srv::AddFact::Request>();
    add_request->fact = pred_msg;

    if (!this->add_fact_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "add_fact service not available in apply_effect");
      return;
    }

    auto add_future = this->add_fact_client_->async_send_request(add_request);
    add_future.wait_for(5s);

  } else {
    // Remove fact
    auto remove_request =
        std::make_shared<easy_plan_msgs::srv::RemoveFact::Request>();
    remove_request->fact = pred_msg;

    if (!this->remove_fact_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "remove_fact service not available in apply_effect");
      return;
    }

    auto remove_future =
        this->remove_fact_client_->async_send_request(remove_request);
    remove_future.wait_for(5s);
  }
}

void KbPddlManager::knowledge_update_callback(
    const easy_plan_msgs::msg::KnowledgeUpdate::SharedPtr msg) {

  // If a goal was added, notify waiting threads
  if (msg->entity_type == easy_plan_msgs::msg::KnowledgeUpdate::GOAL &&
      msg->operation == easy_plan_msgs::msg::KnowledgeUpdate::ADD) {
    this->goal_cv_.notify_all();
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(KbPddlManager, easy_plan::PddlManager)
