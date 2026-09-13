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
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "omni_plan_dispatcher/parallel_plan_dispatcher.hpp"
#include "omni_plan_msgs/msg/plan_action_status.hpp"
#include "omni_plan_msgs/msg/plan_execution_status.hpp"

#include <pluginlib/class_list_macros.hpp>

using namespace omni_plan_dispatcher;
using namespace omni_plan;

ParallelPlanDispatcher::ParallelPlanDispatcher() : omni_plan::PlanDispatcher() {
  this->add_ros_parameters(
      {{"execution_threads",
        static_cast<int>(std::thread::hardware_concurrency()),
        this->execution_threads_}});
}

pddl::ActionStatus ParallelPlanDispatcher::dispatch_actions(
    const std::vector<pddl::GraphNode::Ptr> &all_nodes) {

  const int total = static_cast<int>(all_nodes.size());

  if (total == 0) {
    return pddl::ActionStatus::SUCCEEDED;
  }

  // Defensive validation: node_num is used to index per-node state, so every
  // node must have a unique node_num in [0, total). The base dispatcher
  // validates this too, but dispatch_actions must not rely on it.
  {
    std::vector<bool> seen(total, false);
    for (const auto &node : all_nodes) {
      if (!node || node->node_num < 0 || node->node_num >= total) {
        RCLCPP_ERROR(
            this->node_->get_logger(),
            "[ParallelPlanDispatcher] node_num %d out of range [0, %d)",
            node ? node->node_num : -1, total);
        return pddl::ActionStatus::ABORTED;
      }
      if (seen[node->node_num]) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "[ParallelPlanDispatcher] duplicate node_num %d",
                     node->node_num);
        return pddl::ActionStatus::ABORTED;
      }
      seen[node->node_num] = true;
    }
  }

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

  // Pool size capped so thread count stays constant regardless of plan length
  // and never exceeds the number of nodes. Workers are never blocked in .get()
  // — only in action->run() (I/O-bound).
  const int configured_threads =
      this->execution_threads_ <= 0
          ? static_cast<int>(std::thread::hardware_concurrency())
          : this->execution_threads_;
  const unsigned int pool_size = static_cast<unsigned int>(
      std::min(std::max(configured_threads, 1), total));
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

        // No exception may escape a worker: it would call std::terminate and
        // leave `outstanding` incremented, hanging dispatch_actions forever.
        try {
          fn();
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->node_->get_logger(),
                       "[ParallelPlanDispatcher] Worker task threw: %s",
                       e.what());
        } catch (...) {
          RCLCPP_ERROR(this->node_->get_logger(),
                       "[ParallelPlanDispatcher] Worker task threw an unknown "
                       "exception");
        }

        if (outstanding.fetch_sub(1) == 1) {
          std::lock_guard<std::mutex> lk(all_done_mtx);
          all_done_cv.notify_one();
        }
      }
    });
  }

  // Recursively resolve the promises of a node's whole subtree as SKIPPED.
  // Resolving every descendant (not only direct children) guarantees that
  // results[i].get() can never block, whatever the node numbering is.
  std::function<void(const pddl::GraphNode::Ptr &)> skip_subtree;
  skip_subtree = [&](const pddl::GraphNode::Ptr &node) {
    const int idx = node->node_num;
    try {
      promises[idx].set_value(pddl::ActionStatus::SKIPPED);
    } catch (const std::future_error &) {
    }
    this->set_node_status(idx, omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
    for (const auto &child : node->out_arcs) {
      skip_subtree(child);
    }
  };

  // submit_node enqueues a node once all its deps have resolved.
  // Declared as std::function to allow self-referencing capture.
  std::function<void(const pddl::GraphNode::Ptr &)> submit_node;

  submit_node = [&](const pddl::GraphNode::Ptr &node) {
    submit([this, node, &results, &promises, &pending, &submit_node,
            &skip_subtree]() {
      const int idx = node->node_num;

      // Resolve this node's promise exactly once, whatever path is taken.
      auto resolve = [&promises, idx](pddl::ActionStatus status) {
        try {
          promises[idx].set_value(status);
        } catch (const std::future_error &) {
        }
      };

      try {
        // Dep futures are already resolved here — .get() is non-blocking.
        bool deps_ok = true;
        for (const auto &dep : node->in_arcs) {
          if (results[dep->node_num].get() != pddl::ActionStatus::SUCCEEDED) {
            deps_ok = false;
          }
        }

        if (!deps_ok || this->is_canceled()) {
          this->set_node_status(idx,
                                omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
          resolve(pddl::ActionStatus::SKIPPED);
          this->publish_exec_status(
              omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

        } else {

          auto action = node->action.action;

          if (!action) {
            RCLCPP_ERROR(this->node_->get_logger(),
                         "[ParallelPlanDispatcher] No plugin found for action "
                         "at node %d",
                         idx);

            this->set_node_status(
                idx, omni_plan_msgs::msg::PlanActionStatus::FAILED);
            this->publish_exec_status(
                omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
            resolve(pddl::ActionStatus::ABORTED);

          } else {
            this->set_node_status(
                idx, omni_plan_msgs::msg::PlanActionStatus::RUNNING);
            this->publish_exec_status(
                omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

            std::shared_ptr<pddl::Action> exec_action =
                this->push_current_action(action, true);

            if (!exec_action) {
              // Acquisition failed. If the plan was cancelled concurrently,
              // report cancellation instead of a failure.
              const bool canceled = this->is_canceled();
              this->set_node_status(
                  idx, canceled
                           ? omni_plan_msgs::msg::PlanActionStatus::CANCELLED
                           : omni_plan_msgs::msg::PlanActionStatus::FAILED);
              this->publish_exec_status(
                  omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
              resolve(canceled ? pddl::ActionStatus::CANCELED
                               : pddl::ActionStatus::ABORTED);

            } else if (this->is_canceled()) {
              // The plan was cancelled after this node registered its
              // instance but before it started running. Only the instance
              // acquired for this node may be cancelled, and the node is
              // skipped without executing.
              exec_action->cancel();
              this->remove_current_action(exec_action);
              if (exec_action != action) {
                this->release_cached_action(exec_action);
              }

              this->set_node_status(
                  idx, omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
              this->publish_exec_status(
                  omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
              resolve(pddl::ActionStatus::SKIPPED);

            } else {
              pddl::ActionStatus result =
                  this->run_node_action(node, exec_action);
              this->remove_current_action(exec_action);

              // Return non-primary instances to the cache for future reuse
              if (exec_action != action) {
                this->release_cached_action(exec_action);
              }

              if (result == pddl::ActionStatus::SUCCEEDED) {
                this->set_node_status(
                    idx, omni_plan_msgs::msg::PlanActionStatus::SUCCEEDED);
              } else if (result == pddl::ActionStatus::CANCELED) {
                this->set_node_status(
                    idx, omni_plan_msgs::msg::PlanActionStatus::CANCELLED);
              } else if (result == pddl::ActionStatus::SKIPPED) {
                this->set_node_status(
                    idx, omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
              } else {
                this->set_node_status(
                    idx, omni_plan_msgs::msg::PlanActionStatus::FAILED);
              }

              this->publish_exec_status(
                  omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

              resolve(result);
            }
          }
        }

        if (this->cancel_on_abort_ &&
            results[idx].get() == pddl::ActionStatus::ABORTED) {
          this->cancel_plan();
          for (const auto &child : node->out_arcs) {
            skip_subtree(child);
          }
        } else {
          // Submit children whose last pending dependency just resolved.
          for (const auto &child : node->out_arcs) {
            if (pending[child->node_num].fetch_sub(1) == 1) {
              submit_node(child);
            }
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "[ParallelPlanDispatcher] Task for node %d threw: %s", idx,
                     e.what());
        this->cancel_plan();
        this->set_node_status(idx,
                              omni_plan_msgs::msg::PlanActionStatus::FAILED);
        this->publish_exec_status(
            omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
        resolve(pddl::ActionStatus::ABORTED);
        for (const auto &child : node->out_arcs) {
          skip_subtree(child);
        }
      } catch (...) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "[ParallelPlanDispatcher] Task for node %d threw an "
                     "unknown exception",
                     idx);
        this->cancel_plan();
        this->set_node_status(idx,
                              omni_plan_msgs::msg::PlanActionStatus::FAILED);
        this->publish_exec_status(
            omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
        resolve(pddl::ActionStatus::ABORTED);
        for (const auto &child : node->out_arcs) {
          skip_subtree(child);
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

  this->clear_current_actions();

  // Aggregate outcome: ABORTED > CANCELED > SUCCEEDED. A plan that was
  // cancelled must never be reported as SUCCEEDED, even when no node had to
  // report CANCELED because all remaining nodes were skipped.
  bool any_cancel = false;

  for (int i = 0; i < total; ++i) {
    const pddl::ActionStatus &r = results[i].get();
    if (r == pddl::ActionStatus::ABORTED) {
      return pddl::ActionStatus::ABORTED;
    }
    if (r == pddl::ActionStatus::CANCELED) {
      any_cancel = true;
    }
  }

  if (any_cancel || this->is_canceled()) {
    return pddl::ActionStatus::CANCELED;
  }

  return pddl::ActionStatus::SUCCEEDED;
}

PLUGINLIB_EXPORT_CLASS(ParallelPlanDispatcher, omni_plan::PlanDispatcher)
