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
#include <functional>
#include <future>
#include <queue>
#include <set>

#include "omni_plan/plan_dispatcher.hpp"

using namespace omni_plan;

PlanDispatcher::PlanDispatcher(rclcpp::Node::SharedPtr node)
    : utils::ParameterLoader("plan_dispatcher"),
      action_state_loader_("omni_plan", "omni_plan::pddl::Action") {

  this->add_ros_parameters(
      {{"cancel_on_abort", false, this->cancel_on_abort_},
       {"cancel_on_new_goals", false, this->cancel_on_new_goals_},
       {"execution_threads",
        static_cast<int>(std::thread::hardware_concurrency()),
        this->execution_threads_}});

  this->node_ = node;
  auto qos = rclcpp::QoS(10).reliable();
  this->exec_status_pub_ =
      node->create_publisher<omni_plan_msgs::msg::PlanExecutionStatus>(
          "/omni_plan/plan_execution", qos);
}

pddl::ActionStatus PlanDispatcher::dispatch_plan(
    const std::vector<pddl::GraphNode::Ptr> &all_nodes,
    const std::unordered_map<std::string, std::shared_ptr<pddl::Action>>
        &actions_map,
    const std::unordered_map<std::string, std::string> &actions_plugins_map,
    const std::shared_ptr<PddlManager> &pddl_manager) {

  // Reset cancellation flag from any previous dispatch
  this->is_canceled_.store(false, std::memory_order_relaxed);

  // Initialize execution status
  const int total = static_cast<int>(all_nodes.size());
  this->exec_start_time_ = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
    this->exec_node_status_.resize(total);
    for (const auto &node : all_nodes) {
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
    const std::set<pddl::Predicate> initial_goals =
        pddl_manager->get_pddl().second.get_goals();
    monitor_thread = std::thread([this, &pddl_manager, &monitor_stop,
                                  initial_goals]() {
      while (!monitor_stop.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (monitor_stop.load(std::memory_order_relaxed)) {
          return;
        }
        const auto current_goals = pddl_manager->get_pddl().second.get_goals();
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

  auto result = this->execute_branches(all_nodes, pddl_manager, actions_map,
                                       actions_plugins_map);

  monitor_stop.store(true, std::memory_order_relaxed);
  if (monitor_thread.joinable()) {
    monitor_thread.join();
  }

  // Publish final execution status
  uint8_t final_overall;
  if (result == pddl::ActionStatus::SUCCEED) {
    final_overall = omni_plan_msgs::msg::PlanExecutionStatus::SUCCEEDED;
  } else if (result == pddl::ActionStatus::CANCEL) {
    final_overall = omni_plan_msgs::msg::PlanExecutionStatus::CANCELLED;
  } else {
    final_overall = omni_plan_msgs::msg::PlanExecutionStatus::FAILED;
  }
  this->publish_exec_status(final_overall);
  return result;
}

void PlanDispatcher::cancel_plan() {
  {
    std::lock_guard<std::mutex> lock(this->actions_mutex_);
    for (auto &action : this->current_actions_) {
      if (action) {
        action->cancel();
      }
    }
  }
  this->is_canceled_.store(true, std::memory_order_relaxed);
}

bool PlanDispatcher::is_canceled() const {
  return this->is_canceled_.load(std::memory_order_relaxed);
}

std::shared_ptr<pddl::Action>
PlanDispatcher::acquire_cached_action(const std::string &action_name,
                                      const std::string &plugin_name) {
  {
    std::lock_guard<std::mutex> lk(this->action_cache_mutex_);
    auto &pool = this->action_cache_[action_name];
    if (!pool.empty()) {
      auto cached = pool.back();
      pool.pop_back();
      return cached;
    }
  }
  auto new_action = std::shared_ptr<pddl::Action>(
      this->action_state_loader_.createUnmanagedInstance(plugin_name));
  new_action->load_ros_parameters(this->node_);
  return new_action;
}

void PlanDispatcher::release_cached_action(
    const std::string &action_name, std::shared_ptr<pddl::Action> action) {
  std::lock_guard<std::mutex> lk(this->action_cache_mutex_);
  this->action_cache_[action_name].push_back(std::move(action));
}

void PlanDispatcher::publish_exec_status(uint8_t overall) {
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
      inst_args.push_back(params[action->get_parameter_index(arg)]);
    }
    result.emplace_back(eff.get_type(), eff.get_name(), inst_args,
                        eff.is_negated());
  }
  return result;
}

std::vector<pddl::Effect> PlanDispatcher::apply_effects(
    const std::vector<pddl::Effect> &effects,
    const std::shared_ptr<pddl::Action> &action,
    const std::vector<std::string> &params,
    const std::shared_ptr<PddlManager> &pddl_manager) {
  auto instantiated = instantiate_effects(effects, action, params);
  return pddl_manager->apply_effects(instantiated);
}

void PlanDispatcher::undo_effects(
    const std::vector<pddl::Effect> &effects,
    const std::shared_ptr<PddlManager> &pddl_manager) {
  std::vector<pddl::Effect> reversed = effects;
  for (auto &eff : reversed) {
    eff.set_negation(!eff.is_negated());
  }
  pddl_manager->apply_effects(reversed);
}

pddl::ActionStatus PlanDispatcher::run_node_action(
    const pddl::GraphNode::Ptr &node,
    const std::shared_ptr<pddl::Action> &action,
    const std::shared_ptr<PddlManager> &pddl_manager) {

  const auto &params = node->action.params;

  std::string param_str;
  for (const auto &p : params) {
    param_str += p + " ";
  }
  RCLCPP_INFO(this->node_->get_logger(),
              "Executing action: %s with parameters: %s",
              action->get_name().c_str(), param_str.c_str());

  // Apply start and overall effects under lock
  std::vector<pddl::Effect> on_start_effects;
  std::vector<pddl::Effect> overall_effects;
  {
    std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
    on_start_effects = apply_effects(action->get_on_start_effects(), action,
                                     params, pddl_manager);
    overall_effects = apply_effects(action->get_over_all_effects(), action,
                                    params, pddl_manager);
  }

  // Run the action (blocking, no lock held)
  pddl::ActionStatus status = pddl::ActionStatus::SUCCEED;

  try {
    status = action->run(params);
  } catch (...) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Exception thrown during execution of action '%s'",
                 action->get_name().c_str());
    status = pddl::ActionStatus::ABORT;
  }

  // Undo overall effects and apply end effects under lock
  {
    std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
    undo_effects(overall_effects, pddl_manager);

    if (status == pddl::ActionStatus::SUCCEED) {
      RCLCPP_INFO(this->node_->get_logger(), "Action '%s' succeeded",
                  action->get_name().c_str());
      apply_effects(action->get_on_end_effects(), action, params, pddl_manager);
    } else {
      undo_effects(on_start_effects, pddl_manager);
    }
  }

  if (this->is_canceled() && status == pddl::ActionStatus::CANCEL) {
    RCLCPP_INFO(this->node_->get_logger(), "Plan execution canceled");
    return pddl::ActionStatus::CANCEL;
  }

  if (status == pddl::ActionStatus::ABORT ||
      (!this->is_canceled() && status == pddl::ActionStatus::CANCEL)) {
    RCLCPP_ERROR(this->node_->get_logger(), "Action '%s' aborted",
                 action->get_name().c_str());
    return pddl::ActionStatus::ABORT;
  }

  return pddl::ActionStatus::SUCCEED;
}

pddl::ActionStatus PlanDispatcher::execute_branches(
    const std::vector<pddl::GraphNode::Ptr> &all_nodes,
    std::shared_ptr<PddlManager> pddl_manager,
    const std::unordered_map<std::string, std::shared_ptr<pddl::Action>>
        &actions_map,
    std::unordered_map<std::string, std::string> actions_plugins_map) {

  const int total = static_cast<int>(all_nodes.size());

  // Per-node outcome futures; shared so multiple children can call .get().
  std::vector<std::promise<pddl::ActionStatus>> promises(total);
  std::vector<std::shared_future<pddl::ActionStatus>> results(total);
  for (int i = 0; i < total; ++i) {
    results[i] = promises[i].get_future().share();
  }

  // A node is only submitted to the pool when this counter reaches 0,
  // i.e. every one of its dependencies has already resolved its promise.
  // This guarantees that .get() inside a worker is always non-blocking.
  std::vector<std::atomic<int>> pending(total);
  for (const auto &node : all_nodes) {
    pending[node->node_num].store(static_cast<int>(node->in_arcs.size()));
  }

  // Size is capped at hardware_concurrency so thread count stays constant
  // regardless of plan length.  Workers are never blocked in .get() (see
  // above), only in action->run() — truly I/O-bound work.
  const unsigned int pool_size =
      std::max(1u, this->execution_threads_ <= 0
                       ? std::thread::hardware_concurrency()
                       : static_cast<unsigned int>(this->execution_threads_));
  std::queue<std::function<void()>> task_queue;
  std::mutex queue_mtx;
  std::condition_variable queue_cv;
  bool pool_stop = false;

  std::atomic<int> outstanding{0};
  std::condition_variable all_done_cv;
  std::mutex all_done_mtx;

  auto submit = [&](std::function<void()> fn) {
    outstanding.fetch_add(1);
    {
      std::lock_guard<std::mutex> lk(queue_mtx);
      task_queue.push(std::move(fn));
    }
    queue_cv.notify_one();
  };

  std::vector<std::thread> workers;
  workers.reserve(pool_size);

  for (unsigned w = 0; w < pool_size; ++w) {
    workers.emplace_back([&]() {
      for (;;) {
        std::function<void()> fn;
        {
          std::unique_lock<std::mutex> lk(queue_mtx);
          queue_cv.wait(lk, [&] { return !task_queue.empty() || pool_stop; });
          if (pool_stop && task_queue.empty()) {
            return;
          }
          fn = std::move(task_queue.front());
          task_queue.pop();
        }

        fn();

        if (outstanding.fetch_sub(1) == 1) {
          all_done_cv.notify_one();
        }
      }
    });
  }

  // submit_node enqueues a node once all its deps have resolved.
  // Declared as std::function to allow self-referencing capture.
  std::function<void(const pddl::GraphNode::Ptr &)> submit_node;

  submit_node = [&](const pddl::GraphNode::Ptr &node) {
    submit([this, node, &results, &promises, &pending, &submit_node,
            &actions_map, &actions_plugins_map, pddl_manager]() {
      const int idx = node->node_num;

      // Dep futures are already resolved here — .get() is non-blocking.
      bool deps_ok = true;
      for (const auto &dep : node->in_arcs) {
        if (results[dep->node_num].get() != pddl::ActionStatus::SUCCEED) {
          deps_ok = false;
        }
      }

      if (!deps_ok || this->is_canceled()) {
        {
          std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
          this->exec_node_status_[idx].status =
              omni_plan_msgs::msg::PlanActionStatus::SKIPPED;
        }
        promises[idx].set_value(pddl::ActionStatus::SKIP);
        this->publish_exec_status(
            omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

      } else {

        auto it = actions_map.find(node->action.action->get_name());
        auto action = (it != actions_map.end() ? it->second : nullptr);

        if (!action) {
          RCLCPP_ERROR(this->node_->get_logger(),
                       "No plugin found for action '%s'",
                       node->action.action->get_name().c_str());

          {
            std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
            this->exec_node_status_[idx].status =
                omni_plan_msgs::msg::PlanActionStatus::FAILED;
            this->exec_node_status_[idx].wall_end = this->node_->now();
          }

          this->publish_exec_status(
              omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
          promises[idx].set_value(pddl::ActionStatus::ABORT);

        } else {
          {
            std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
            this->exec_node_status_[idx].status =
                omni_plan_msgs::msg::PlanActionStatus::RUNNING;
            this->exec_node_status_[idx].wall_start = this->node_->now();
          }

          this->publish_exec_status(
              omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

          // Acquire the instance to run: use the primary if it is idle,
          // otherwise get a cached idle instance (or create one) so that
          // parallel branches never share a single Action object.
          const std::string action_name = node->action.action->get_name();
          const std::string action_plugin = actions_plugins_map[action_name];
          std::shared_ptr<pddl::Action> exec_action;
          {
            std::lock_guard<std::mutex> lk(this->actions_mutex_);
            auto it = std::find(this->current_actions_.begin(),
                                this->current_actions_.end(), action);
            if (it != this->current_actions_.end()) {
              // Primary instance is busy; acquire from cache or create new
              exec_action =
                  this->acquire_cached_action(action_name, action_plugin);
            } else {
              exec_action = action;
            }

            this->current_actions_.push_back(exec_action);
          }

          pddl::ActionStatus result =
              this->run_node_action(node, exec_action, pddl_manager);

          {
            std::lock_guard<std::mutex> lk(this->actions_mutex_);
            this->current_actions_.erase(
                std::remove(this->current_actions_.begin(),
                            this->current_actions_.end(), exec_action),
                this->current_actions_.end());
          }

          // Return non-primary instances to the cache for future reuse
          if (exec_action != action) {
            this->release_cached_action(action_name, exec_action);
          }

          {
            std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
            auto &s = this->exec_node_status_[idx];
            if (result == pddl::ActionStatus::SUCCEED) {
              s.status = omni_plan_msgs::msg::PlanActionStatus::SUCCEEDED;
            } else if (result == pddl::ActionStatus::CANCEL) {
              s.status = omni_plan_msgs::msg::PlanActionStatus::CANCELLED;
            } else {
              s.status = omni_plan_msgs::msg::PlanActionStatus::FAILED;
            }
            s.wall_end = this->node_->now();
          }

          this->publish_exec_status(
              omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

          promises[idx].set_value(result);
        }
      }

      if (this->cancel_on_abort_ &&
          results[idx].get() == pddl::ActionStatus::ABORT) {
        this->cancel_plan();

      } else {
        // Submit children whose last pending dependency just resolved.
        for (const auto &child : node->out_arcs) {
          if (pending[child->node_num].fetch_sub(1) == 1) {
            submit_node(child);
          }
        }
      }
    });
  };

  // Enqueue root nodes (no dependencies → pending already 0).
  for (const auto &node : all_nodes) {
    if (node->in_arcs.empty()) {
      submit_node(node);
    }
  }

  // Wait until every node has finished.
  {
    std::unique_lock<std::mutex> lk(all_done_mtx);
    all_done_cv.wait(lk, [&] { return outstanding.load() == 0; });
  }

  // Shut down the pool.
  {
    std::lock_guard<std::mutex> lk(queue_mtx);
    pool_stop = true;
  }

  queue_cv.notify_all();
  for (auto &w : workers) {
    if (w.joinable()) {
      w.join();
    }
  }

  {
    std::lock_guard<std::mutex> lk(this->actions_mutex_);
    this->current_actions_.clear();
  }

  // Aggregate outcome: ABORT > CANCEL > SUCCEED.
  bool any_cancel = false;
  for (int i = 0; i < total; ++i) {
    const pddl::ActionStatus &r = results[i].get();
    if (r == pddl::ActionStatus::ABORT) {
      return pddl::ActionStatus::ABORT;
    }
    if (r == pddl::ActionStatus::CANCEL) {
      any_cancel = true;
    }
  }
  return any_cancel ? pddl::ActionStatus::CANCEL : pddl::ActionStatus::SUCCEED;
}
