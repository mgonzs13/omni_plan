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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/timing_predicate.hpp"
#include "omni_plan/pddl_manager.hpp"
#include "omni_plan_dispatcher/parallel_plan_dispatcher.hpp"
#include "omni_plan_dispatcher/sequential_plan_dispatcher.hpp"
#include "omni_plan_msgs/msg/plan_action_status.hpp"

using namespace omni_plan;
using namespace omni_plan::pddl;

// =============================================================================
// Mock helpers
// =============================================================================

/**
 * @brief Controllable mock action for dispatcher tests.
 */
class MockDispatcherAction : public Action {
public:
  enum class RunResult { SUCCEED, CANCEL, ABORT };

  explicit MockDispatcherAction(const std::string &name,
                                RunResult result = RunResult::SUCCEED)
      : Action(name, 10.0f, {}), run_result_(result), run_count_(0),
        canceled_(false) {}

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    ++run_count_;
    if (canceled_.load()) {
      return ActionStatus::CANCELED;
    }
    switch (run_result_) {
    case RunResult::SUCCEED:
      return ActionStatus::SUCCEEDED;
    case RunResult::CANCEL:
      return ActionStatus::CANCELED;
    case RunResult::ABORT:
      return ActionStatus::ABORTED;
    }
    return ActionStatus::SUCCEEDED;
  }

  void cancel() override { canceled_.store(true); }

  int get_run_count() const { return run_count_.load(); }

  RunResult run_result_;
  std::atomic<int> run_count_;
  std::atomic<bool> canceled_;
};

/**
 * @brief Action that invokes a user-supplied callback before returning.
 * Returns CANCELED if cancel() was called, SUCCEEDED otherwise.
 * Use this to drive cancel_plan() from inside a running action.
 */
class CallbackAction : public Action {
public:
  explicit CallbackAction(const std::string &name,
                          std::function<void()> cb = {})
      : Action(name, 10.0f, {}), on_run_(std::move(cb)), canceled_(false),
        run_count_(0) {}

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    ++run_count_;
    if (on_run_) {
      on_run_();
    }
    return canceled_.load() ? ActionStatus::CANCELED : ActionStatus::SUCCEEDED;
  }

  void cancel() override { canceled_.store(true); }

  int get_run_count() const { return run_count_.load(); }

  std::function<void()> on_run_;
  std::atomic<bool> canceled_;
  std::atomic<int> run_count_;
};

/**
 * @brief Minimal PddlManager implementation for testing.
 * Records how many times effects are applied for assertion.
 */
class MockPddlManager : public PddlManager {
public:
  MockPddlManager() : PddlManager(), effect_apply_count_(0) {}

  std::pair<pddl::Domain, pddl::Problem> get_pddl() const override {
    return {pddl::Domain(), pddl::Problem()};
  }

  bool has_goals() const override { return false; }
  bool clear_goals() const override { return true; }

  bool predicate_exists(const pddl::Predicate & /*pred*/) const override {
    return false;
  }

  bool predicate_is_goal(const pddl::Predicate & /*pred*/) const override {
    return false;
  }

  void apply_effect(const pddl::Effect & /*eff*/) override {
    ++effect_apply_count_;
  }

  int get_effect_apply_count() const { return effect_apply_count_.load(); }

private:
  std::atomic<int> effect_apply_count_;
};

/**
 * @brief PddlManager whose apply_effect always throws, used to exercise the
 * dispatcher's exception handling inside worker threads.
 */
class ThrowingPddlManager : public MockPddlManager {
public:
  void apply_effect(const pddl::Effect & /*eff*/) override {
    throw std::runtime_error("apply_effect failed");
  }
};

/**
 * @brief Action that always succeeds but triggers a user callback on run().
 * cancel() is intentionally a no-op so the plan can be cancelled while this
 * action reports SUCCEEDED.
 */
class CancelTriggerAction : public Action {
public:
  explicit CancelTriggerAction(const std::string &name,
                               std::function<void()> cb = {})
      : Action(name, 10.0f, {}), on_run_(std::move(cb)), run_count_(0) {}

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    ++run_count_;
    if (on_run_) {
      on_run_();
    }
    return ActionStatus::SUCCEEDED;
  }

  void cancel() override {}

  int get_run_count() const { return run_count_.load(); }

  std::function<void()> on_run_;
  std::atomic<int> run_count_;
};

/**
 * @brief Action that blocks in run() until cancel() is invoked (or a timeout
 * elapses). Used to exercise cancellation racing with an in-flight action.
 */
class BlockingCancelAction : public Action {
public:
  explicit BlockingCancelAction(const std::string &name,
                                std::atomic<bool> *started = nullptr)
      : Action(name, 10.0f, {}), started_(started), canceled_(false) {}

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    if (started_ != nullptr) {
      started_->store(true);
    }
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait_for(lk, std::chrono::seconds(5), [&] { return canceled_.load(); });
    return canceled_.load() ? ActionStatus::CANCELED : ActionStatus::SUCCEEDED;
  }

  void cancel() override {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      canceled_.store(true);
    }
    cv_.notify_all();
  }

private:
  std::atomic<bool> *started_;
  std::atomic<bool> canceled_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

// =============================================================================
// Graph-building utilities
// =============================================================================

static GraphNode::Ptr make_node(int node_num, int level_num,
                                std::shared_ptr<Action> action,
                                std::vector<std::string> params = {}) {
  auto node = GraphNode::make_shared();
  node->node_num = node_num;
  node->level_num = level_num;
  node->action.action = action;
  node->action.params = std::move(params);
  return node;
}

/// Adds a dependency edge: child must execute after parent.
static void link_nodes(GraphNode::Ptr parent, GraphNode::Ptr child) {
  parent->out_arcs.push_back(child);
  child->in_arcs.push_back(parent);
}

// =============================================================================
// Test fixture
// =============================================================================

class PlanDispatcherTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_dispatcher_node");
    pddl_manager_ = std::make_shared<MockPddlManager>();
  }

  /// Creates and initialises a SequentialPlanDispatcher.
  std::shared_ptr<omni_plan_dispatcher::SequentialPlanDispatcher>
  make_sequential() {
    auto d = std::make_shared<omni_plan_dispatcher::SequentialPlanDispatcher>();
    d->initialize(node_, pddl_manager_);
    return d;
  }

  /// Creates and initialises a ParallelPlanDispatcher.
  std::shared_ptr<omni_plan_dispatcher::ParallelPlanDispatcher>
  make_parallel() {
    auto d = std::make_shared<omni_plan_dispatcher::ParallelPlanDispatcher>();
    d->initialize(node_, pddl_manager_);
    return d;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MockPddlManager> pddl_manager_;
};

// =============================================================================
// SequentialPlanDispatcher tests
// =============================================================================

TEST_F(PlanDispatcherTest, Sequential_EmptyPlan_Succeeds) {
  auto d = make_sequential();
  std::vector<GraphNode::Ptr> nodes;
  EXPECT_EQ(d->dispatch_plan(nodes), ActionStatus::SUCCEEDED);
}

TEST_F(PlanDispatcherTest, Sequential_SingleSuccessfulAction_Succeeds) {
  auto action = std::make_shared<MockDispatcherAction>("move");
  auto node = make_node(0, 0, action);

  auto d = make_sequential();
  EXPECT_EQ(d->dispatch_plan({node}), ActionStatus::SUCCEEDED);
  EXPECT_EQ(action->get_run_count(), 1);
}

TEST_F(PlanDispatcherTest, Sequential_SingleAbortingAction_ReturnsAborted) {
  auto action = std::make_shared<MockDispatcherAction>(
      "move", MockDispatcherAction::RunResult::ABORT);
  auto node = make_node(0, 0, action);

  auto d = make_sequential();
  EXPECT_EQ(d->dispatch_plan({node}), ActionStatus::ABORTED);
}

TEST_F(PlanDispatcherTest, Sequential_TwoLinearActions_BothExecuted) {
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n1 = make_node(0, 0, a1);
  auto n2 = make_node(1, 1, a2);
  link_nodes(n1, n2);

  auto d = make_sequential();
  EXPECT_EQ(d->dispatch_plan({n1, n2}), ActionStatus::SUCCEEDED);
  EXPECT_EQ(a1->get_run_count(), 1);
  EXPECT_EQ(a2->get_run_count(), 1);
}

TEST_F(PlanDispatcherTest,
       Sequential_FirstActionAborts_SecondNotExecuted_CancelOnAbortFalse) {
  auto a1 = std::make_shared<MockDispatcherAction>(
      "pick", MockDispatcherAction::RunResult::ABORT);
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n1 = make_node(0, 0, a1);
  auto n2 = make_node(1, 1, a2);
  link_nodes(n1, n2);

  auto d = make_sequential();
  EXPECT_EQ(d->dispatch_plan({n1, n2}), ActionStatus::ABORTED);
  // Second action must not run after the first one aborts.
  EXPECT_EQ(a2->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest,
       Sequential_FirstActionAborts_CancelOnAbortTrue_ReturnsAborted) {
  auto a1 = std::make_shared<MockDispatcherAction>(
      "pick", MockDispatcherAction::RunResult::ABORT);
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n1 = make_node(0, 0, a1);
  auto n2 = make_node(1, 1, a2);
  link_nodes(n1, n2);

  // Derive to expose protected field
  struct TestableSeq : omni_plan_dispatcher::SequentialPlanDispatcher {
    void set_cancel_on_abort(bool v) { cancel_on_abort_ = v; }
  };
  auto d = std::make_shared<TestableSeq>();
  d->initialize(node_, pddl_manager_);
  d->set_cancel_on_abort(true);

  EXPECT_EQ(d->dispatch_plan({n1, n2}), ActionStatus::ABORTED);
  EXPECT_EQ(a2->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest,
       Sequential_ThreeInOrderByLevels_ExecutedInLevelOrder) {
  // Actions are in reverse node_num order but have correct level numbers.
  // Dispatcher must sort by (level_num, node_num), not by order in the vector.
  auto a0 = std::make_shared<MockDispatcherAction>("a0");
  auto a1 = std::make_shared<MockDispatcherAction>("a1");
  auto a2 = std::make_shared<MockDispatcherAction>("a2");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 1, a1);
  auto n2 = make_node(2, 2, a2);

  // Pass them in shuffled order
  auto d = make_sequential();
  EXPECT_EQ(d->dispatch_plan({n2, n0, n1}), ActionStatus::SUCCEEDED);
  EXPECT_EQ(a0->get_run_count(), 1);
  EXPECT_EQ(a1->get_run_count(), 1);
  EXPECT_EQ(a2->get_run_count(), 1);
}

TEST_F(PlanDispatcherTest,
       Sequential_CancellationDuringFirstAction_SecondSkipped) {
  auto d = make_sequential();

  // a1 triggers cancel_plan() while it is running; the dispatcher will then
  // call cancel() back on a1, causing it to return CANCELED.
  auto a1 =
      std::make_shared<CallbackAction>("pick", [&d]() { d->cancel_plan(); });
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n1 = make_node(0, 0, a1);
  auto n2 = make_node(1, 1, a2);

  EXPECT_EQ(d->dispatch_plan({n1, n2}), ActionStatus::CANCELED);
  EXPECT_EQ(a2->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest, Sequential_Abort_MarksRemainingNodesSkipped) {
  struct TestableSeq : omni_plan_dispatcher::SequentialPlanDispatcher {
    const std::vector<omni_plan_msgs::msg::PlanActionStatus> &statuses() const {
      return exec_node_status_;
    }
  };

  auto a1 = std::make_shared<MockDispatcherAction>(
      "pick", MockDispatcherAction::RunResult::ABORT);
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n1 = make_node(0, 0, a1);
  auto n2 = make_node(1, 1, a2);
  link_nodes(n1, n2);

  auto d = std::make_shared<TestableSeq>();
  d->initialize(node_, pddl_manager_);
  EXPECT_EQ(d->dispatch_plan({n1, n2}), ActionStatus::ABORTED);

  const auto &statuses = d->statuses();
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[0].status, omni_plan_msgs::msg::PlanActionStatus::FAILED);
  EXPECT_EQ(statuses[1].status, omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
}

TEST_F(PlanDispatcherTest, Sequential_Cancel_MarksRemainingNodesSkipped) {
  struct TestableSeq : omni_plan_dispatcher::SequentialPlanDispatcher {
    const std::vector<omni_plan_msgs::msg::PlanActionStatus> &statuses() const {
      return exec_node_status_;
    }
  };

  auto d = std::make_shared<TestableSeq>();
  d->initialize(node_, pddl_manager_);

  auto a1 =
      std::make_shared<CallbackAction>("pick", [&d]() { d->cancel_plan(); });
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n1 = make_node(0, 0, a1);
  auto n2 = make_node(1, 1, a2);
  link_nodes(n1, n2);

  EXPECT_EQ(d->dispatch_plan({n1, n2}), ActionStatus::CANCELED);
  EXPECT_EQ(a2->get_run_count(), 0);

  const auto &statuses = d->statuses();
  ASSERT_EQ(statuses.size(), 2u);
  EXPECT_EQ(statuses[0].status,
            omni_plan_msgs::msg::PlanActionStatus::CANCELLED);
  EXPECT_EQ(statuses[1].status, omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
}

// =============================================================================
// ParallelPlanDispatcher tests
// =============================================================================

TEST_F(PlanDispatcherTest, Parallel_EmptyPlan_Succeeds) {
  auto d = make_parallel();
  std::vector<GraphNode::Ptr> nodes;
  EXPECT_EQ(d->dispatch_plan(nodes), ActionStatus::SUCCEEDED);
}

TEST_F(PlanDispatcherTest, Parallel_SingleSuccessfulAction_Succeeds) {
  auto action = std::make_shared<MockDispatcherAction>("move");
  auto node = make_node(0, 0, action);

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({node}), ActionStatus::SUCCEEDED);
  EXPECT_EQ(action->get_run_count(), 1);
}

TEST_F(PlanDispatcherTest, Parallel_SingleAbortingAction_ReturnsAborted) {
  auto action = std::make_shared<MockDispatcherAction>(
      "move", MockDispatcherAction::RunResult::ABORT);
  auto node = make_node(0, 0, action);

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({node}), ActionStatus::ABORTED);
}

TEST_F(PlanDispatcherTest, Parallel_TwoIndependentActions_BothExecuted) {
  auto a0 = std::make_shared<MockDispatcherAction>("move");
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 0, a1);
  // No dependency between them → both are roots.

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({n0, n1}), ActionStatus::SUCCEEDED);
  EXPECT_EQ(a0->get_run_count(), 1);
  EXPECT_EQ(a1->get_run_count(), 1);
}

TEST_F(PlanDispatcherTest, Parallel_DependentActions_BothSucceed) {
  auto a0 = std::make_shared<MockDispatcherAction>("move");
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 1, a1);
  link_nodes(n0, n1);

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({n0, n1}), ActionStatus::SUCCEEDED);
  EXPECT_EQ(a0->get_run_count(), 1);
  EXPECT_EQ(a1->get_run_count(), 1);
}

TEST_F(PlanDispatcherTest,
       Parallel_ParentAborts_DependentChildIsSkipped_ReturnsAborted) {
  auto a0 = std::make_shared<MockDispatcherAction>(
      "move", MockDispatcherAction::RunResult::ABORT);
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 1, a1);
  link_nodes(n0, n1);

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({n0, n1}), ActionStatus::ABORTED);
  // Child must be skipped (deps not ok), not run.
  EXPECT_EQ(a1->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest,
       Parallel_CancelOnAbortTrue_NoDeadlock_ReturnsAborted) {
  // Verifies that enabling cancel_on_abort does not deadlock when a node
  // aborts and has downstream dependents whose promises must still resolve.
  struct TestablePar : omni_plan_dispatcher::ParallelPlanDispatcher {
    void set_cancel_on_abort(bool v) { cancel_on_abort_ = v; }
  };

  auto a0 = std::make_shared<MockDispatcherAction>(
      "move", MockDispatcherAction::RunResult::ABORT);
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 1, a1);
  auto n2 = make_node(2, 2, a2);
  link_nodes(n0, n1);
  link_nodes(n1, n2);

  auto d = std::make_shared<TestablePar>();
  d->initialize(node_, pddl_manager_);
  d->set_cancel_on_abort(true);

  // This call must return (not deadlock) and report ABORTED.
  EXPECT_EQ(d->dispatch_plan({n0, n1, n2}), ActionStatus::ABORTED);
  EXPECT_EQ(a1->get_run_count(), 0);
  EXPECT_EQ(a2->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest, Parallel_TwoRoots_OneAborts_ReturnsAborted) {
  auto a0 = std::make_shared<MockDispatcherAction>(
      "move", MockDispatcherAction::RunResult::ABORT);
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  // Two independent roots; a0 aborts, a1 succeeds.
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 0, a1);

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({n0, n1}), ActionStatus::ABORTED);
}

TEST_F(PlanDispatcherTest,
       Parallel_CancellationDuringExecution_DependentsSkipped) {
  auto d = make_parallel();

  // n0 triggers cancel_plan() while running; n1 depends on n0 and must be
  // skipped because n0 returns CANCELED.
  auto a0 =
      std::make_shared<CallbackAction>("move", [&d]() { d->cancel_plan(); });
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 1, a1);
  link_nodes(n0, n1);

  EXPECT_EQ(d->dispatch_plan({n0, n1}), ActionStatus::CANCELED);
  EXPECT_EQ(a1->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest, Parallel_DiamondDependency_AllSucceed) {
  // Diamond dependency: n0 -> (n1, n2) -> n3
  auto a0 = std::make_shared<MockDispatcherAction>("a0");
  auto a1 = std::make_shared<MockDispatcherAction>("a1");
  auto a2 = std::make_shared<MockDispatcherAction>("a2");
  auto a3 = std::make_shared<MockDispatcherAction>("a3");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 1, a1);
  auto n2 = make_node(2, 1, a2);
  auto n3 = make_node(3, 2, a3);
  link_nodes(n0, n1);
  link_nodes(n0, n2);
  link_nodes(n1, n3);
  link_nodes(n2, n3);

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({n0, n1, n2, n3}), ActionStatus::SUCCEEDED);
  EXPECT_EQ(a0->get_run_count(), 1);
  EXPECT_EQ(a1->get_run_count(), 1);
  EXPECT_EQ(a2->get_run_count(), 1);
  EXPECT_EQ(a3->get_run_count(), 1);
}

TEST_F(PlanDispatcherTest,
       Parallel_EffectPathThrows_ReturnsAbortedNotTerminate) {
  // apply_effects() runs outside run_node_action()'s try/catch. An exception
  // thrown there must be contained in the worker and reported as ABORTED.
  auto action = std::make_shared<MockDispatcherAction>("move");
  action->add_effect(Type::START, "at", {"x"});
  auto node = make_node(0, 0, action);

  auto throwing_manager = std::make_shared<ThrowingPddlManager>();
  auto d = std::make_shared<omni_plan_dispatcher::ParallelPlanDispatcher>();
  d->initialize(node_, throwing_manager);

  EXPECT_EQ(d->dispatch_plan({node}), ActionStatus::ABORTED);
}

TEST_F(PlanDispatcherTest, Parallel_CancelWithNoRunningAction_ReturnsCanceled) {
  auto d = make_parallel();

  // a0 succeeds while cancelling the plan; the remaining nodes are skipped
  // without any node ever reporting CANCELED.
  auto a0 = std::make_shared<CancelTriggerAction>("move",
                                                  [&d]() { d->cancel_plan(); });
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto a2 = std::make_shared<MockDispatcherAction>("place");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 1, a1);
  auto n2 = make_node(2, 2, a2);
  link_nodes(n0, n1);
  link_nodes(n1, n2);

  EXPECT_EQ(d->dispatch_plan({n0, n1, n2}), ActionStatus::CANCELED);
  EXPECT_EQ(a1->get_run_count(), 0);
  EXPECT_EQ(a2->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest,
       Parallel_CancelOnAbort_NonTopologicalNumbering_ResolvesDescendants) {
  // The ABORTED node has the highest index, so the aggregate loop reaches the
  // unresolved grandchild (index 1) before the abort (index 2). Every skipped
  // descendant must have its promise resolved to avoid blocking forever.
  struct TestablePar : omni_plan_dispatcher::ParallelPlanDispatcher {
    void set_cancel_on_abort(bool v) { cancel_on_abort_ = v; }
  };

  auto a_root = std::make_shared<MockDispatcherAction>(
      "root", MockDispatcherAction::RunResult::ABORT);
  auto a_child = std::make_shared<MockDispatcherAction>("child");
  auto a_grandchild = std::make_shared<MockDispatcherAction>("grandchild");

  auto root = make_node(2, 0, a_root);
  auto child = make_node(0, 1, a_child);
  auto grandchild = make_node(1, 2, a_grandchild);
  link_nodes(root, child);
  link_nodes(child, grandchild);

  auto d = std::make_shared<TestablePar>();
  d->initialize(node_, pddl_manager_);
  d->set_cancel_on_abort(true);

  EXPECT_EQ(d->dispatch_plan({child, grandchild, root}), ActionStatus::ABORTED);
  EXPECT_EQ(a_child->get_run_count(), 0);
  EXPECT_EQ(a_grandchild->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest, Parallel_DuplicateNodeNumbers_ReturnsAborted) {
  auto a0 = std::make_shared<MockDispatcherAction>("move");
  auto a1 = std::make_shared<MockDispatcherAction>("pick");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(0, 0, a1);

  auto d = make_parallel();
  EXPECT_EQ(d->dispatch_plan({n0, n1}), ActionStatus::ABORTED);
  EXPECT_EQ(a0->get_run_count(), 0);
  EXPECT_EQ(a1->get_run_count(), 0);
}

TEST_F(PlanDispatcherTest,
       Parallel_CancelWhileActionRunning_ReturnsCanceledNoHang) {
  auto d = make_parallel();
  std::atomic<bool> started{false};

  auto a0 = std::make_shared<BlockingCancelAction>("move", &started);
  auto a1 = std::make_shared<BlockingCancelAction>("pick");
  auto n0 = make_node(0, 0, a0);
  auto n1 = make_node(1, 0, a1);

  ActionStatus result = ActionStatus::ABORTED;
  std::thread runner([&] { result = d->dispatch_plan({n0, n1}); });

  while (!started.load()) {
    std::this_thread::yield();
  }
  d->cancel_plan();
  runner.join();

  EXPECT_EQ(result, ActionStatus::CANCELED);
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
