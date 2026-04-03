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
#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"

#include "poirot/poirot.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl_manager.hpp"

#include "omni_plan_msgs/msg/plan_execution_status.hpp"

class ExecutePlanState : public yasmin::State {

public:
  ExecutePlanState()
      : yasmin::State({
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::ABORT,
            yasmin_ros::basic_outcomes::CANCEL,
        }),
        action_class_loader_("omni_plan", "omni_plan::pddl::Action") {

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

    this->build_plugin_cache(blackboard);

    // Build initial predicates from the current world state by checking
    // ALL conditions (start + over_all + end) of every plan action.
    // Using only on_start_conditions would miss OVER_ALL predicates (e.g.
    // battery_full), causing is_action_executable() to wrongly reject root
    // nodes and produce an empty graph → premature SUCCEED.
    std::set<omni_plan::pddl::Predicate> initial_predicates;
    for (size_t i = 0; i < plan.size(); ++i) {
      auto [action, params] = plan.get_action_with_params(i);
      for (const auto &cond : action->get_conditions()) {
        auto args = cond.get_args();
        std::vector<std::string> inst_args;
        for (const auto &arg : args) {
          int idx = action->get_parameter_index(arg);
          if (idx >= 0 && idx < static_cast<int>(params.size())) {
            inst_args.push_back(params[idx]);
          } else {
            inst_args.push_back(arg);
          }
        }
        omni_plan::pddl::Predicate pred(cond.get_name(), inst_args,
                                        cond.is_negated());
        if (!cond.is_negated() && pddl_manager->predicate_exists(pred)) {
          initial_predicates.insert(pred);
        }
      }
    }

    // Build the planning graph
    omni_plan::pddl::PlanningGraphBuilder builder(initial_predicates);
    auto graph = builder.build_graph(plan);

    // Collect all graph nodes
    auto all_nodes = this->collect_nodes(graph);
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
        for (const auto &dep : node->in_arcs) {
          s.depends_on.push_back(static_cast<int32_t>(dep->node_num));
        }
        s.status = omni_plan_msgs::msg::PlanActionStatus::PENDING;
      }
    }
    this->publish_exec_status(
        omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

    // Branch parallel execution: each node runs as soon as its
    // dependencies complete, maximizing parallelism across branches
    auto result = this->execute_branches(all_nodes, pddl_manager);

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
  pluginlib::ClassLoader<omni_plan::pddl::Action> action_class_loader_;
  std::unordered_map<std::string, std::string> action_to_plugin_;
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

  void build_plugin_cache(yasmin::Blackboard::SharedPtr blackboard) {
    if (!this->action_to_plugin_.empty()) {
      return;
    }
    auto plugins = blackboard->get<std::vector<std::string>>("actions_plugins");
    for (const auto &plugin_class : plugins) {
      if (plugin_class.empty()) {
        continue;
      }
      auto temp = std::shared_ptr<omni_plan::pddl::Action>(
          this->action_class_loader_.createUnmanagedInstance(plugin_class));
      this->action_to_plugin_[temp->get_name()] = plugin_class;
    }
  }

  std::shared_ptr<omni_plan::pddl::Action>
  create_action_instance(const std::string &action_name) {
    auto it = this->action_to_plugin_.find(action_name);
    if (it == this->action_to_plugin_.end()) {
      return nullptr;
    }
    auto instance = std::shared_ptr<omni_plan::pddl::Action>(
        this->action_class_loader_.createUnmanagedInstance(it->second));
    instance->load_ros_parameters(yasmin_ros::YasminNode::get_instance());
    return instance;
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
                std::shared_ptr<omni_plan::PddlManager> pddl_manager) {
    auto instantiated = instantiate_effects(effects, action, params);
    return pddl_manager->apply_effects(instantiated);
  }

  static void
  undo_effects(const std::vector<omni_plan::pddl::Effect> &effects,
               std::shared_ptr<omni_plan::PddlManager> pddl_manager) {
    std::vector<omni_plan::pddl::Effect> reversed = effects;
    for (auto &eff : reversed) {
      eff.set_negation(!eff.is_negated());
    }
    pddl_manager->apply_effects(reversed);
  }

  std::string
  run_node_action(const omni_plan::pddl::GraphNode::Ptr &node,
                  const std::shared_ptr<omni_plan::pddl::Action> &action,
                  std::shared_ptr<omni_plan::PddlManager> pddl_manager) {

    const auto &params = node->action.params;

    std::string param_str;
    for (const auto &p : params) {
      param_str += p + " ";
    }
    YASMIN_LOG_INFO("Executing action: %s with parameters: %s",
                    action->get_name().c_str(), param_str.c_str());

    // Apply start and overall effects under lock
    std::vector<omni_plan::pddl::Effect> overall_effects;
    {
      std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
      apply_effects(action->get_on_start_effects(), action, params,
                    pddl_manager);
      overall_effects = apply_effects(action->get_over_all_effects(), action,
                                      params, pddl_manager);
    }

    // Run the action (blocking, no lock held)
    auto status = action->run(params);

    // Undo overall effects and apply end effects under lock
    {
      std::lock_guard<std::mutex> lock(this->pddl_manager_mutex_);
      undo_effects(overall_effects, pddl_manager);

      if (status == omni_plan::pddl::ActionStatus::SUCCEED) {
        YASMIN_LOG_INFO("Action '%s' succeeded", action->get_name().c_str());
        apply_effects(action->get_on_end_effects(), action, params,
                      pddl_manager);
      }
    }

    if (this->is_canceled() &&
        status == omni_plan::pddl::ActionStatus::CANCEL) {
      YASMIN_LOG_INFO("Plan execution canceled");
      return yasmin_ros::basic_outcomes::CANCEL;
    }
    if (status == omni_plan::pddl::ActionStatus::ABORT ||
        (!this->is_canceled() &&
         status == omni_plan::pddl::ActionStatus::CANCEL)) {
      YASMIN_LOG_ERROR("Action '%s' aborted", action->get_name().c_str());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    return yasmin_ros::basic_outcomes::SUCCEED;
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
      std::shared_ptr<omni_plan::PddlManager> pddl_manager) {

    // Pending dependency count per node, indexed directly by node_num
    std::vector<std::atomic<int>> pending_deps(all_nodes.size());
    for (const auto &node : all_nodes) {
      pending_deps[node->node_num].store(
          static_cast<int>(node->in_arcs.size()));
    }

    // Shared execution state
    std::atomic<int> running{0};
    std::atomic<bool> has_failure{false};
    std::string failure_outcome;
    std::mutex outcome_mutex;
    std::condition_variable done_cv;
    std::mutex done_mutex;

    // Thread storage
    std::list<std::thread> threads;
    std::mutex threads_mutex;

    // Recursive skip: propagate failure/cancel through unexecuted descendants
    std::function<void(const omni_plan::pddl::GraphNode::Ptr &)>
        skip_descendants;
    skip_descendants = [&](const omni_plan::pddl::GraphNode::Ptr &node) {
      for (const auto &child : node->out_arcs) {
        if (pending_deps[child->node_num].fetch_sub(1) == 1) {
          {
            std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
            this->exec_node_status_[child->node_num].status =
                omni_plan_msgs::msg::PlanActionStatus::SKIPPED;
          }
          skip_descendants(child);
        }
      }
    };

    // Branch execution: each node runs as soon as all its dependencies
    // complete, then triggers its children immediately
    std::function<void(omni_plan::pddl::GraphNode::Ptr)> execute_node;
    execute_node = [&](omni_plan::pddl::GraphNode::Ptr node) {
      // Skip execution if already failed or canceled
      if (has_failure.load() || this->is_canceled()) {
        {
          std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
          this->exec_node_status_[node->node_num].status =
              omni_plan_msgs::msg::PlanActionStatus::SKIPPED;
        }
        skip_descendants(node);
        running.fetch_sub(1);
        done_cv.notify_one();
        return;
      }

      // Create a fresh action instance for this node
      auto action =
          this->create_action_instance(node->action.action->get_name());

      // Mark as RUNNING and publish status update
      auto running_time = yasmin_ros::YasminNode::get_instance()->now();
      {
        std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
        auto &s_run = this->exec_node_status_[node->node_num];
        s_run.status = omni_plan_msgs::msg::PlanActionStatus::RUNNING;
        s_run.wall_start = running_time;
      }
      this->publish_exec_status(
          omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

      // Register for cancellation
      {
        std::lock_guard<std::mutex> lock(this->actions_mutex_);
        this->current_actions_.push_back(action);
      }

      // Execute the action
      std::string result = this->run_node_action(node, action, pddl_manager);

      // Unregister from cancellation
      {
        std::lock_guard<std::mutex> lock(this->actions_mutex_);
        this->current_actions_.erase(std::remove(this->current_actions_.begin(),
                                                 this->current_actions_.end(),
                                                 action),
                                     this->current_actions_.end());
      }

      // Update node status based on result
      auto done_time = yasmin_ros::YasminNode::get_instance()->now();
      {
        std::lock_guard<std::mutex> lock(this->exec_node_status_mutex_);
        auto &s = this->exec_node_status_[node->node_num];
        if (result == yasmin_ros::basic_outcomes::SUCCEED) {
          s.status = omni_plan_msgs::msg::PlanActionStatus::SUCCEEDED;
        } else if (result == yasmin_ros::basic_outcomes::CANCEL) {
          s.status = omni_plan_msgs::msg::PlanActionStatus::CANCELLED;
        } else {
          s.status = omni_plan_msgs::msg::PlanActionStatus::FAILED;
        }
        s.wall_end = done_time;
      }
      this->publish_exec_status(
          omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

      // Handle failure
      if (result != yasmin_ros::basic_outcomes::SUCCEED) {
        has_failure.store(true);
        std::lock_guard<std::mutex> lock(outcome_mutex);
        if (failure_outcome.empty()) {
          failure_outcome = result;
        }
      }

      // Trigger ready children or propagate skip on failure
      for (const auto &child : node->out_arcs) {
        if (pending_deps[child->node_num].fetch_sub(1) == 1) {
          if (!has_failure.load() && !this->is_canceled()) {
            running.fetch_add(1);
            std::lock_guard<std::mutex> lock(threads_mutex);
            threads.emplace_back(execute_node, child);
          } else {
            skip_descendants(child);
          }
        }
      }

      running.fetch_sub(1);
      done_cv.notify_one();
    };

    // Launch root nodes (nodes with no dependencies)
    {
      std::lock_guard<std::mutex> lock(threads_mutex);
      for (const auto &node : all_nodes) {
        if (node->in_arcs.empty()) {
          running.fetch_add(1);
          threads.emplace_back(execute_node, node);
        }
      }
    }

    // Wait for all branches to complete
    {
      std::unique_lock<std::mutex> lock(done_mutex);
      done_cv.wait(lock, [&] { return running.load() <= 0; });
    }

    // Join all threads
    {
      std::lock_guard<std::mutex> lock(threads_mutex);
      for (auto &t : threads) {
        if (t.joinable()) {
          t.join();
        }
      }
    }

    // Clear current actions
    {
      std::lock_guard<std::mutex> lock(actions_mutex_);
      this->current_actions_.clear();
    }

    if (has_failure.load()) {
      std::lock_guard<std::mutex> lock(outcome_mutex);
      return failure_outcome;
    }

    if (is_canceled()) {
      return yasmin_ros::basic_outcomes::CANCEL;
    }

    return yasmin_ros::basic_outcomes::SUCCEED;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ExecutePlanState, yasmin::State)