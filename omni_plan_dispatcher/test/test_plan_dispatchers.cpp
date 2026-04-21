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
#include <memory>
#include <mutex>
#include <string>
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
