// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include <set>

#include "omni_plan/plan_dispatcher.hpp"

using namespace omni_plan;

PlanDispatcher::PlanDispatcher()
    : utils::ParameterLoader("plan_dispatcher"),
      action_state_loader_("omni_plan", "omni_plan::pddl::Action") {

  this->add_ros_parameters(
      {{"cancel_on_abort", false, this->cancel_on_abort_},
       {"cancel_on_new_goals", false, this->cancel_on_new_goals_}});
}

void PlanDispatcher::initialize(rclcpp::Node::SharedPtr node,
                                std::shared_ptr<PddlManager> pddl_manager) {
  this->node_ = node;
  this->pddl_manager_ = pddl_manager;
  auto qos = rclcpp::QoS(10).reliable();
  this->exec_status_pub_ =
      node->create_publisher<omni_plan_msgs::msg::PlanExecutionStatus>(
          "/omni_plan/plan_execution", qos);
}

pddl::ActionStatus PlanDispatcher::dispatch_plan(
    const std::vector<pddl::GraphNode::Ptr> &all_nodes) {

  // Reset cancellation flag from any previous dispatch
  this->is_canceled_.store(false, std::memory_order_relaxed);

  // Initialise per-node execution status
  if (all_nodes.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Plan too large: %zu nodes exceeds int max", all_nodes.size());
    return pddl::ActionStatus::ABORTED;
  }

  const int total = static_cast<int>(all_nodes.size());
  this->exec_start_time_ = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
    this->exec_node_status_.resize(total);
    for (const auto &node : all_nodes) {
      if (!node->action.action) {
        RCLCPP_ERROR(this->node_->get_logger(), "Node %d has no action plugin",
                     node->node_num);
        return pddl::ActionStatus::ABORTED;
      }

      auto &s = this->exec_node_status_[node->node_num];
      s.action_name = node->action.action->get_name();
      s.parameters = node->action.params;
      s.node_id = static_cast<int32_t>(node->node_num);
      s.level = static_cast<int32_t>(node->level_num);
      s.depends_on.clear();
      for (const auto &dep : node->in_arcs) {
        s.depends_on.push_back(static_cast<int32_t>(dep->node_num));
      }
      s.status = omni_plan_msgs::msg::PlanActionStatus::PENDING;
      s.wall_start = builtin_interfaces::msg::Time();
      s.wall_end = builtin_interfaces::msg::Time();
    }
  }
  this->publish_exec_status(omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

  // If cancel_on_new_goals is set, a monitor thread watches for new goals
  // added to the PDDL manager and cancels execution when found.
  // A snapshot of the goal set is taken here so that pre-existing goals do
  // not trigger a cancel.
  std::atomic<bool> monitor_stop{false};
  std::thread monitor_thread;
  if (this->cancel_on_new_goals_) {
    std::set<pddl::Predicate> initial_goals;
    {
      std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
      initial_goals = this->pddl_manager_->get_pddl().second.get_goals();
    }

    monitor_thread = std::thread([this, &monitor_stop, initial_goals]() {
      while (!monitor_stop.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (monitor_stop.load(std::memory_order_relaxed)) {
          return;
        }

        std::set<pddl::Predicate> current_goals;
        {
          std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
          current_goals = this->pddl_manager_->get_pddl().second.get_goals();
        }

        for (const auto &goal : current_goals) {
          if (initial_goals.find(goal) == initial_goals.end()) {
            RCLCPP_INFO(this->node_->get_logger(),
                        "New goals detected during execution, cancelling plan");
            this->cancel_plan();
            return;
          }
        }
      }
    });
  }

  // Delegate to the concrete strategy implementation
  auto result = this->dispatch_actions(all_nodes);

  monitor_stop.store(true, std::memory_order_relaxed);
  if (monitor_thread.joinable()) {
    monitor_thread.join();
  }

  // Publish final execution status
  uint8_t final_overall;
  if (result == pddl::ActionStatus::SUCCEEDED) {
    final_overall = omni_plan_msgs::msg::PlanExecutionStatus::SUCCEEDED;
  } else if (result == pddl::ActionStatus::CANCELED) {
    final_overall = omni_plan_msgs::msg::PlanExecutionStatus::CANCELLED;
  } else {
    final_overall = omni_plan_msgs::msg::PlanExecutionStatus::FAILED;
  }
  this->publish_exec_status(final_overall);
  return result;
}

void PlanDispatcher::cancel_plan() {
  std::vector<std::shared_ptr<pddl::Action>> actions_copy;
  {
    std::lock_guard<std::mutex> lock(this->actions_mutex_);
    actions_copy = this->current_actions_;
  }

  this->is_canceled_.store(true, std::memory_order_relaxed);
  for (auto &action : actions_copy) {
    if (action) {
      action->cancel();
    }
  }
}

bool PlanDispatcher::is_canceled() const {
  return this->is_canceled_.load(std::memory_order_relaxed);
}

std::shared_ptr<pddl::Action>
PlanDispatcher::acquire_cached_action(std::shared_ptr<pddl::Action> action) {
  {
    std::lock_guard<std::mutex> lk(this->action_cache_mutex_);
    auto &pool = this->action_cache_[action->get_name()];
    if (!pool.empty()) {
      auto cached = pool.back();
      pool.pop_back();
      return cached;
    }
  }

  pddl::Action *raw = this->action_state_loader_.createUnmanagedInstance(
      action->get_plugin_name());
  if (!raw) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Failed to create action plugin '%s'",
                 action->get_plugin_name().c_str());
    return nullptr;
  }

  auto new_action = std::shared_ptr<pddl::Action>(raw);
  new_action->load_ros_parameters(this->node_);
  return new_action;
}

void PlanDispatcher::release_cached_action(
    std::shared_ptr<pddl::Action> action) {
  std::lock_guard<std::mutex> lk(this->action_cache_mutex_);
  this->action_cache_[action->get_name()].push_back(std::move(action));
}

std::shared_ptr<pddl::Action>
PlanDispatcher::push_current_action(std::shared_ptr<pddl::Action> action,
                                    bool use_cache) {
  if (!use_cache) {
    std::lock_guard<std::mutex> lk(this->actions_mutex_);
    this->current_actions_.push_back(action);
    return action;

  } else {
    // Acquire the instance to run: use the primary if it is idle,
    // otherwise get a cached idle instance (or create one) so that
    // parallel branches never share a single Action object.
    std::shared_ptr<pddl::Action> exec_action;
    {
      std::lock_guard<std::mutex> lk(this->actions_mutex_);
      auto it = std::find(this->current_actions_.begin(),
                          this->current_actions_.end(), action);
      if (it != this->current_actions_.end()) {
        exec_action = this->acquire_cached_action(action);
      } else {
        exec_action = action;
      }
      this->current_actions_.push_back(exec_action);
    }
    return exec_action;
  }
}

void PlanDispatcher::remove_current_action(
    std::shared_ptr<pddl::Action> action) {
  std::lock_guard<std::mutex> lk(this->actions_mutex_);
  this->current_actions_.erase(std::remove(this->current_actions_.begin(),
                                           this->current_actions_.end(),
                                           action),
                               this->current_actions_.end());
}

void PlanDispatcher::clear_current_actions() {
  std::lock_guard<std::mutex> lk(this->actions_mutex_);
  this->current_actions_.clear();
}

void PlanDispatcher::set_node_status(int node_num, uint8_t status) {
  std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
  if (node_num >= 0 &&
      node_num < static_cast<int>(this->exec_node_status_.size())) {
    this->exec_node_status_[node_num].status = status;

    if (status == omni_plan_msgs::msg::PlanActionStatus::RUNNING) {
      this->exec_node_status_[node_num].wall_start = this->node_->now();
    } else if (status == omni_plan_msgs::msg::PlanActionStatus::SUCCEEDED ||
               status == omni_plan_msgs::msg::PlanActionStatus::FAILED ||
               status == omni_plan_msgs::msg::PlanActionStatus::CANCELLED) {
      this->exec_node_status_[node_num].wall_end = this->node_->now();
    }
  }
}

void PlanDispatcher::publish_exec_status(uint8_t overall) {
  if (!this->exec_status_pub_) {
    return;
  }

  omni_plan_msgs::msg::PlanExecutionStatus msg;
  auto now = std::chrono::steady_clock::now();
  msg.elapsed_time =
      std::chrono::duration<double>(now - this->exec_start_time_).count();
  msg.overall_status = overall;
  msg.stamp = this->node_->now();

  {
    std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
    msg.actions = this->exec_node_status_;
  }

  this->exec_status_pub_->publish(msg);
}

std::vector<pddl::Effect>
PlanDispatcher::instantiate_effects(const std::vector<pddl::Effect> &effects,
                                    const std::shared_ptr<pddl::Action> &action,
                                    const std::vector<std::string> &params) {
  std::vector<pddl::Effect> result;
  result.reserve(effects.size());
  for (const auto &eff : effects) {
    auto args = eff.get_args();
    std::vector<std::string> inst_args;
    inst_args.reserve(args.size());

    for (const auto &arg : args) {
      int idx = action->get_parameter_index(arg);
      if (idx >= 0 && idx < static_cast<int>(params.size())) {
        inst_args.push_back(params[idx]);
      } else {
        inst_args.push_back(arg);
      }
    }

    result.emplace_back(eff.get_type(), eff.get_name(), inst_args,
                        eff.is_negated());
  }
  return result;
}

std::vector<pddl::Effect>
PlanDispatcher::apply_effects(const std::vector<pddl::Effect> &effects,
                              const std::shared_ptr<pddl::Action> &action,
                              const std::vector<std::string> &params) {
  std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
  auto instantiated = instantiate_effects(effects, action, params);
  return this->pddl_manager_->apply_effects(instantiated);
}

void PlanDispatcher::undo_effects(const std::vector<pddl::Effect> &effects) {
  std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
  std::vector<pddl::Effect> reversed = effects;
  for (auto &eff : reversed) {
    eff.set_negation(!eff.is_negated());
  }
  this->pddl_manager_->apply_effects(reversed);
}

pddl::ActionStatus
PlanDispatcher::run_node_action(const pddl::GraphNode::Ptr &node,
                                const std::shared_ptr<pddl::Action> &action) {

  const auto &params = node->action.params;

  std::string param_str;
  for (const auto &p : params) {
    param_str += p + " ";
  }
  RCLCPP_INFO(this->node_->get_logger(),
              "Executing action: %s with parameters: %s",
              action->get_name().c_str(), param_str.c_str());

  // Apply start effects under lock
  std::vector<pddl::Effect> on_start_effects;

  on_start_effects =
      this->apply_effects(action->get_on_start_effects(), action, params);

  // Run the action (blocking, no lock held)
  pddl::ActionStatus status = pddl::ActionStatus::SUCCEEDED;

  try {
    status = action->run(params);
  } catch (...) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Exception thrown during execution of action '%s'",
                 action->get_name().c_str());
    status = pddl::ActionStatus::ABORTED;
  }

  if (status == pddl::ActionStatus::SUCCEEDED) {
    RCLCPP_INFO(this->node_->get_logger(), "Action '%s' succeeded",
                action->get_name().c_str());
    this->apply_effects(action->get_on_end_effects(), action, params);
  } else {
    this->undo_effects(on_start_effects);
  }

  if (this->is_canceled() && status == pddl::ActionStatus::CANCELED) {
    RCLCPP_INFO(this->node_->get_logger(), "Plan execution canceled");
    return pddl::ActionStatus::CANCELED;
  }

  if (status == pddl::ActionStatus::ABORTED ||
      (!this->is_canceled() && status == pddl::ActionStatus::CANCELED)) {
    RCLCPP_ERROR(this->node_->get_logger(), "Action '%s' aborted",
                 action->get_name().c_str());
    return pddl::ActionStatus::ABORTED;
  }

  return pddl::ActionStatus::SUCCEEDED;
}
