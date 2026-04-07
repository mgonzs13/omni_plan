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

#include <algorithm>
#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "poirot/poirot.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/pddl_manager.hpp"

#include "omni_plan_msgs/msg/plan_execution_status.hpp"

class ExecutePlanState : public yasmin::State {

public:
  ExecutePlanState()
      : yasmin::State({
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::ABORT,
            yasmin_ros::basic_outcomes::CANCEL,
        }) {
    this->set_description(
        "State responsible for executing a given plan, which is expected to be "
        "on the blackboard. The state will execute each action in the plan as "
        "soon as all its dependencies have succeeded, maximizing parallelism "
        "across branches. The state monitors for cancellation and aborts "
        "execution as soon as possible if requested.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::SUCCEED,
                                  "Plan executed successfully");
    this->set_outcome_description(yasmin_ros::basic_outcomes::CANCEL,
                                  "Plan execution was cancelled");
    this->set_outcome_description(yasmin_ros::basic_outcomes::ABORT,
                                  "Plan execution failed due to an error");
    this->add_input_key(
        "plan", "The plan to execute, expected to be on the blackboard");
    this->add_input_key("actions", "Map of action name to Action plugin, "
                                   "expected to be on the blackboard");
    this->add_input_key(
        "problem",
        "The PDDL problem, expected to be on the blackboard (used for initial "
        "state predicates)");
    this->add_input_key("pddl_manager",
                        "The PDDL manager, expected to be on the blackboard");
  }

  void configure() override {
    auto node = yasmin_ros::YasminNode::get_instance();
    auto qos = rclcpp::QoS(10).reliable();
    this->exec_status_pub_ =
        node->create_publisher<omni_plan_msgs::msg::PlanExecutionStatus>(
            "/omni_plan/plan_execution", qos);
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    PROFILE_FUNCTION();

    auto pddl_manager =
        blackboard->get<std::shared_ptr<omni_plan::PddlManager>>(
            "pddl_manager");
    auto plan = blackboard->get<omni_plan::pddl::Plan>("plan");
    auto actions_map = blackboard->get<std::unordered_map<
        std::string, std::shared_ptr<omni_plan::pddl::Action>>>("actions");

    // The PDDL problem stored on the blackboard already contains the current
    // world state as its (:init ...) facts — exactly what PlanningGraphBuilder
    // needs.
    auto problem = blackboard->get<omni_plan::pddl::Problem>("problem");
    const std::set<omni_plan::pddl::Predicate> &initial_predicates =
        problem.get_facts();

    // Build the planning graph
    omni_plan::pddl::PlanningGraphBuilder builder(initial_predicates);
    auto graph = builder.build_graph(plan);

    // Collect all graph nodes
    auto all_nodes = this->collect_nodes(graph);

    // Renumber sequentially here so every node_num is a valid index into [0,
    // total)
    for (size_t i = 0; i < all_nodes.size(); ++i) {
      all_nodes[i]->node_num = static_cast<int>(i);
    }

    int total = static_cast<int>(all_nodes.size());

    if (total == 0) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    }

    YASMIN_LOG_INFO("Planning graph built with %d nodes for branch execution",
                    total);

    // Initialize execution status
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
        s.wall_start = {};
        s.wall_end = {};
      }
    }
    this->publish_exec_status(
        omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

    // Branch parallel execution: each node runs as soon as its
    // dependencies complete, maximizing parallelism across branches
    auto result = this->execute_branches(all_nodes, pddl_manager, actions_map);

    // Publish final execution status
    uint8_t final_overall;
    if (result == yasmin_ros::basic_outcomes::SUCCEED) {
      final_overall = omni_plan_msgs::msg::PlanExecutionStatus::SUCCEEDED;
    } else if (result == yasmin_ros::basic_outcomes::CANCEL) {
      final_overall = omni_plan_msgs::msg::PlanExecutionStatus::CANCELLED;
    } else {
      final_overall = omni_plan_msgs::msg::PlanExecutionStatus::FAILED;
    }
    this->publish_exec_status(final_overall);
    return result;
  }

  void cancel_state() override {
    {
      std::lock_guard<std::mutex> lock(this->actions_mutex_);
      for (auto &action : this->current_actions_) {
        if (action) {
          action->cancel();
        }
      }
    }
    yasmin::State::cancel_state();
  }

private:
  std::vector<std::shared_ptr<omni_plan::pddl::Action>> current_actions_;
  std::mutex actions_mutex_;
  std::mutex pddl_manager_mutex_;
  rclcpp::Publisher<omni_plan_msgs::msg::PlanExecutionStatus>::SharedPtr
      exec_status_pub_;
  std::vector<omni_plan_msgs::msg::PlanActionStatus> exec_node_status_;
  std::mutex exec_node_status_mutex_;
  std::chrono::steady_clock::time_point exec_start_time_;

  void publish_exec_status(uint8_t overall) {
    omni_plan_msgs::msg::PlanExecutionStatus msg;
    auto now = std::chrono::steady_clock::now();
    msg.elapsed_time =
        std::chrono::duration<double>(now - this->exec_start_time_).count();
    msg.overall_status = overall;
    msg.stamp = yasmin_ros::YasminNode::get_instance()->now();
    {
      std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
      msg.actions = this->exec_node_status_;
    }
    this->exec_status_pub_->publish(msg);
  }

  std::shared_ptr<omni_plan::pddl::Action>
  get_action(const std::string &name,
             const std::unordered_map<std::string,
                                      std::shared_ptr<omni_plan::pddl::Action>>
                 &actions_map) const {
    auto it = actions_map.find(name);
    if (it == actions_map.end()) {
      return nullptr;
    }
    return it->second;
  }

  static std::vector<omni_plan::pddl::Effect>
  instantiate_effects(const std::vector<omni_plan::pddl::Effect> &effects,
                      const std::shared_ptr<omni_plan::pddl::Action> &action,
                      const std::vector<std::string> &params) {
    std::vector<omni_plan::pddl::Effect> result;
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

  static std::vector<omni_plan::pddl::Effect>
  apply_effects(const std::vector<omni_plan::pddl::Effect> &effects,
                const std::shared_ptr<omni_plan::pddl::Action> &action,
                const std::vector<std::string> &params,
                const std::shared_ptr<omni_plan::PddlManager> &pddl_manager) {
    auto instantiated = instantiate_effects(effects, action, params);
    return pddl_manager->apply_effects(instantiated);
  }

  static void
  undo_effects(const std::vector<omni_plan::pddl::Effect> &effects,
               const std::shared_ptr<omni_plan::PddlManager> &pddl_manager) {
    std::vector<omni_plan::pddl::Effect> reversed = effects;
    for (auto &eff : reversed) {
      eff.set_negation(!eff.is_negated());
    }
    pddl_manager->apply_effects(reversed);
  }

  omni_plan::pddl::ActionStatus
  run_node_action(const omni_plan::pddl::GraphNode::Ptr &node,
                  const std::shared_ptr<omni_plan::pddl::Action> &action,
                  const std::shared_ptr<omni_plan::PddlManager> &pddl_manager) {

    const auto &params = node->action.params;

    std::string param_str;
    for (const auto &p : params) {
      param_str += p + " ";
    }
    YASMIN_LOG_INFO("Executing action: %s with parameters: %s",
                    action->get_name().c_str(), param_str.c_str());

    // Apply start and overall effects under lock
    std::vector<omni_plan::pddl::Effect> on_start_effects;
    std::vector<omni_plan::pddl::Effect> overall_effects;
    {
      std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
      on_start_effects = this->apply_effects(action->get_on_start_effects(),
                                             action, params, pddl_manager);
      overall_effects = this->apply_effects(action->get_over_all_effects(),
                                            action, params, pddl_manager);
    }

    // Run the action (blocking, no lock held)
    auto status = action->run(params);

    // Undo overall effects and apply end effects under lock
    {
      std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
      this->undo_effects(overall_effects, pddl_manager);

      if (status == omni_plan::pddl::ActionStatus::SUCCEED) {
        YASMIN_LOG_INFO("Action '%s' succeeded", action->get_name().c_str());
        this->apply_effects(action->get_on_end_effects(), action, params,
                            pddl_manager);
      } else {
        this->undo_effects(on_start_effects, pddl_manager);
      }
    }

    if (this->is_canceled() &&
        status == omni_plan::pddl::ActionStatus::CANCEL) {
      YASMIN_LOG_INFO("Plan execution canceled");
      return omni_plan::pddl::ActionStatus::CANCEL;
    }
    if (status == omni_plan::pddl::ActionStatus::ABORT ||
        (!this->is_canceled() &&
         status == omni_plan::pddl::ActionStatus::CANCEL)) {
      YASMIN_LOG_ERROR("Action '%s' aborted", action->get_name().c_str());
      return omni_plan::pddl::ActionStatus::ABORT;
    }

    return omni_plan::pddl::ActionStatus::SUCCEED;
  }

  static std::vector<omni_plan::pddl::GraphNode::Ptr>
  collect_nodes(const omni_plan::pddl::PlanningGraph::Ptr &graph) {
    std::vector<omni_plan::pddl::GraphNode::Ptr> nodes;
    std::set<omni_plan::pddl::GraphNode::Ptr> visited;

    std::function<void(const omni_plan::pddl::GraphNode::Ptr &)> traverse;
    traverse = [&](const omni_plan::pddl::GraphNode::Ptr &n) {
      if (!visited.insert(n).second) {
        return;
      }
      nodes.push_back(n);
      for (const auto &child : n->out_arcs) {
        traverse(child);
      }
    };
    for (const auto &root : graph->roots) {
      traverse(root);
    }
    return nodes;
  }

  std::string execute_branches(
      const std::vector<omni_plan::pddl::GraphNode::Ptr> &all_nodes,
      std::shared_ptr<omni_plan::PddlManager> pddl_manager,
      const std::unordered_map<
          std::string, std::shared_ptr<omni_plan::pddl::Action>> &actions_map) {

    const int total = static_cast<int>(all_nodes.size());

    // Per-node outcome futures; shared so multiple children can call .get().
    std::vector<std::promise<std::string>> promises(total);
    std::vector<std::shared_future<std::string>> results(total);
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
        std::max(1u, std::thread::hardware_concurrency());
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
    std::function<void(const omni_plan::pddl::GraphNode::Ptr &)> submit_node;

    submit_node = [&](const omni_plan::pddl::GraphNode::Ptr &node) {
      submit([this, node, &results, &promises, &pending, &submit_node,
              &actions_map, pddl_manager]() {
        const int idx = node->node_num;

        // Dep futures are already resolved here — .get() is non-blocking.
        bool deps_ok = true;
        for (const auto &dep : node->in_arcs) {
          if (results[dep->node_num].get() !=
              yasmin_ros::basic_outcomes::SUCCEED) {
            deps_ok = false;
          }
        }

        if (!deps_ok || this->is_canceled()) {
          {
            std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
            this->exec_node_status_[idx].status =
                omni_plan_msgs::msg::PlanActionStatus::SKIPPED;
          }
          promises[idx].set_value("skipped");
        } else {
          auto action =
              this->get_action(node->action.action->get_name(), actions_map);
          if (!action) {
            YASMIN_LOG_ERROR("No plugin found for action '%s'",
                             node->action.action->get_name().c_str());
            {
              std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
              this->exec_node_status_[idx].status =
                  omni_plan_msgs::msg::PlanActionStatus::FAILED;
              this->exec_node_status_[idx].wall_end =
                  yasmin_ros::YasminNode::get_instance()->now();
            }
            this->publish_exec_status(
                omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
            promises[idx].set_value(yasmin_ros::basic_outcomes::ABORT);
          } else {
            {
              std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
              this->exec_node_status_[idx].status =
                  omni_plan_msgs::msg::PlanActionStatus::RUNNING;
              this->exec_node_status_[idx].wall_start =
                  yasmin_ros::YasminNode::get_instance()->now();
            }
            this->publish_exec_status(
                omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

            {
              std::lock_guard<std::mutex> lk(this->actions_mutex_);
              this->current_actions_.push_back(action);
            }

            omni_plan::pddl::ActionStatus result =
                this->run_node_action(node, action, pddl_manager);

            {
              std::lock_guard<std::mutex> lk(this->actions_mutex_);
              this->current_actions_.erase(
                  std::remove(this->current_actions_.begin(),
                              this->current_actions_.end(), action),
                  this->current_actions_.end());
            }

            {
              std::lock_guard<std::mutex> lk(this->exec_node_status_mutex_);
              auto &s = this->exec_node_status_[idx];
              if (result == omni_plan::pddl::ActionStatus::SUCCEED) {
                s.status = omni_plan_msgs::msg::PlanActionStatus::SUCCEEDED;
              } else if (result == omni_plan::pddl::ActionStatus::CANCEL) {
                s.status = omni_plan_msgs::msg::PlanActionStatus::CANCELLED;
              } else {
                s.status = omni_plan_msgs::msg::PlanActionStatus::FAILED;
              }
              s.wall_end = yasmin_ros::YasminNode::get_instance()->now();
            }
            this->publish_exec_status(
                omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
            promises[idx].set_value(
                result == omni_plan::pddl::ActionStatus::SUCCEED
                    ? yasmin_ros::basic_outcomes::SUCCEED
                    : (result == omni_plan::pddl::ActionStatus::CANCEL
                           ? yasmin_ros::basic_outcomes::CANCEL
                           : yasmin_ros::basic_outcomes::ABORT));
          }
        }

        // Submit children whose last pending dependency just resolved.
        for (const auto &child : node->out_arcs) {
          if (pending[child->node_num].fetch_sub(1) == 1) {
            submit_node(child);
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
      const std::string &r = results[i].get();
      if (r == yasmin_ros::basic_outcomes::ABORT) {
        return yasmin_ros::basic_outcomes::ABORT;
      }
      if (r == yasmin_ros::basic_outcomes::CANCEL) {
        any_cancel = true;
      }
    }
    return any_cancel ? yasmin_ros::basic_outcomes::CANCEL
                      : yasmin_ros::basic_outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ExecutePlanState, yasmin::State)