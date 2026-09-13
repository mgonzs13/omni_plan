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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/timing_predicate.hpp"

using namespace omni_plan::pddl;

// ==================== Mock Action ====================
class MockGraphAction : public Action {
public:
  MockGraphAction(const std::string &name,
                  std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params), run_called_(false) {}

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    run_called_ = true;
    return ActionStatus::SUCCEEDED;
  }

  void cancel() override {}

  bool run_called_;
};

// ==================== PlanningGraphBuilder Tests ====================
class PlanningGraphBuilderTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create actions for a logistics scenario:
    // move(robot, from, to): moves robot from one room to another
    // pick(robot, item, room): picks up an item in a room
    // drop(robot, item, room): drops an item in a room

    move_action_ = std::make_shared<MockGraphAction>(
        "move", std::vector<std::pair<std::string, std::string>>{
                    {"robot", "robot"}, {"r1", "room"}, {"r2", "room"}});
    move_action_->add_condition(START, "robot_at", {"robot", "r1"});
    move_action_->add_condition(OVER_ALL, "battery_full", {"robot"});
    move_action_->add_effect(START, "robot_at", {"robot", "r1"}, true);
    move_action_->add_effect(END, "robot_at", {"robot", "r2"});

    pick_action_ = std::make_shared<MockGraphAction>(
        "pick", std::vector<std::pair<std::string, std::string>>{
                    {"robot", "robot"}, {"item", "item"}, {"room", "room"}});
    pick_action_->add_condition(START, "robot_at", {"robot", "room"});
    pick_action_->add_condition(START, "item_at", {"item", "room"});
    pick_action_->add_effect(START, "item_at", {"item", "room"}, true);
    pick_action_->add_effect(END, "robot_has", {"robot", "item"});

    drop_action_ = std::make_shared<MockGraphAction>(
        "drop", std::vector<std::pair<std::string, std::string>>{
                    {"robot", "robot"}, {"item", "item"}, {"room", "room"}});
    drop_action_->add_condition(START, "robot_at", {"robot", "room"});
    drop_action_->add_condition(START, "robot_has", {"robot", "item"});
    drop_action_->add_effect(START, "robot_has", {"robot", "item"}, true);
    drop_action_->add_effect(END, "item_at", {"item", "room"});

    // Initial state
    initial_predicates_.insert(
        Predicate("robot_at", {"robot1", "room1"}, false));
    initial_predicates_.insert(
        Predicate("robot_at", {"robot2", "room2"}, false));
    initial_predicates_.insert(Predicate("battery_full", {"robot1"}, false));
    initial_predicates_.insert(Predicate("battery_full", {"robot2"}, false));
    initial_predicates_.insert(Predicate("item_at", {"item1", "room1"}, false));
    initial_predicates_.insert(Predicate("item_at", {"item2", "room2"}, false));
  }

  std::shared_ptr<MockGraphAction> move_action_;
  std::shared_ptr<MockGraphAction> pick_action_;
  std::shared_ptr<MockGraphAction> drop_action_;
  std::set<Predicate> initial_predicates_;
};

// ==================== Plan Time Tests ====================
TEST_F(PlanningGraphBuilderTest, PlanStoresStartTime) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.0f);

  EXPECT_FLOAT_EQ(plan.get_action_start_time(0), 0.0f);
  EXPECT_FLOAT_EQ(plan.get_action_start_time(1), 10.0f);
}

TEST_F(PlanningGraphBuilderTest, PlanDefaultTime) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "room1", "room2"});

  EXPECT_FLOAT_EQ(plan.get_action_start_time(0), 0.0f);
}

TEST_F(PlanningGraphBuilderTest, PlanTimeOutOfRange) {
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);

  EXPECT_THROW(plan.get_action_start_time(10), std::out_of_range);
}

// ==================== Graph Builder: Sequential Plan ====================
TEST_F(PlanningGraphBuilderTest, SequentialPlanCreatesLinearGraph) {
  // Plan: move robot1 room1->room2 (0.0), pick item1 at room2 (10.0)
  // pick depends on move (needs robot_at robot1 room2)
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  ASSERT_NE(graph, nullptr);
  // move is a root (executable from initial state)
  EXPECT_EQ(graph->roots.size(), 1u);
  EXPECT_EQ(graph->roots.front()->action.action->get_name(), "move");

  // pick should depend on move
  auto move_node = graph->roots.front();
  EXPECT_EQ(move_node->out_arcs.size(), 1u);
  EXPECT_EQ(move_node->out_arcs.front()->action.action->get_name(), "pick");
}

TEST_F(PlanningGraphBuilderTest, SequentialPlanExecutionLevels) {
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);
  auto levels = PlanningGraphBuilder::get_execution_levels(graph);

  // Two levels: move, then pick
  ASSERT_EQ(levels.size(), 2u);
  EXPECT_EQ(levels[0].size(), 1u);
  EXPECT_EQ(levels[0][0]->action.action->get_name(), "move");
  EXPECT_EQ(levels[1].size(), 1u);
  EXPECT_EQ(levels[1][0]->action.action->get_name(), "pick");
}

// ==================== Graph Builder: Parallel Plan ====================
TEST_F(PlanningGraphBuilderTest, ParallelActionsAtSameTime) {
  // Two robots: robot1 picks item1 at room1, robot2 picks item2 at room2
  // These are independent and should be parallel
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(pick_action_, {"robot1", "item1", "room1"}, 0.0f);
  plan.add_action(pick_action_, {"robot2", "item2", "room2"}, 0.0f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  ASSERT_NE(graph, nullptr);
  // Both should be roots (independent actions)
  EXPECT_EQ(graph->roots.size(), 2u);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  // Single level with two parallel actions
  ASSERT_EQ(levels.size(), 1u);
  EXPECT_EQ(levels[0].size(), 2u);
}

TEST_F(PlanningGraphBuilderTest, ParallelThenSequential) {
  // Two moves at t=0 (parallel), then a pick at t=10
  // robot1: room1->room3, robot2: room2->room4 (parallel at t=0)
  // Then robot1 picks item at room3 (at t=10, depends on move)
  auto move_action2 = std::make_shared<MockGraphAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"robot", "robot"}, {"r1", "room"}, {"r2", "room"}});
  move_action2->add_condition(START, "robot_at", {"robot", "r1"});
  move_action2->add_condition(OVER_ALL, "battery_full", {"robot"});
  move_action2->add_effect(START, "robot_at", {"robot", "r1"}, true);
  move_action2->add_effect(END, "robot_at", {"robot", "r2"});

  initial_predicates_.insert(Predicate("item_at", {"item1", "room3"}, false));

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room3"}, 0.0f);
  plan.add_action(move_action2, {"robot2", "room2", "room4"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room3"}, 10.001f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  ASSERT_NE(graph, nullptr);
  // Two root actions (parallel moves)
  EXPECT_EQ(graph->roots.size(), 2u);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  // Two levels: parallel moves, then sequential pick
  ASSERT_EQ(levels.size(), 2u);
  EXPECT_EQ(levels[0].size(), 2u);
  EXPECT_EQ(levels[1].size(), 1u);
  EXPECT_EQ(levels[1][0]->action.action->get_name(), "pick");
}

// ==================== Graph Builder: Empty Plan ====================
TEST_F(PlanningGraphBuilderTest, EmptyPlanCreatesEmptyGraph) {
  Plan plan;
  plan.set_has_solution(true);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  ASSERT_NE(graph, nullptr);
  EXPECT_EQ(graph->roots.size(), 0u);
  EXPECT_EQ(graph->levels.size(), 0u);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  EXPECT_EQ(levels.size(), 0u);
}

// ==================== Graph Builder: Single Action ====================
TEST_F(PlanningGraphBuilderTest, SingleActionPlan) {
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(pick_action_, {"robot1", "item1", "room1"}, 0.0f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  ASSERT_NE(graph, nullptr);
  EXPECT_EQ(graph->roots.size(), 1u);
  EXPECT_EQ(graph->roots.front()->action.action->get_name(), "pick");
  EXPECT_EQ(graph->roots.front()->in_arcs.size(), 0u);
  EXPECT_EQ(graph->roots.front()->out_arcs.size(), 0u);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  ASSERT_EQ(levels.size(), 1u);
  EXPECT_EQ(levels[0].size(), 1u);
}

// ==================== Graph Builder: Three-Level Chain ====================
TEST_F(PlanningGraphBuilderTest, ThreeLevelChainPlan) {
  // move robot1 room1->room2, pick item at room2, drop item at room2
  // This is a fully sequential chain

  initial_predicates_.insert(Predicate("item_at", {"item1", "room2"}, false));

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);
  plan.add_action(drop_action_, {"robot1", "item1", "room2"}, 20.002f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  ASSERT_NE(graph, nullptr);
  EXPECT_EQ(graph->roots.size(), 1u);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  ASSERT_EQ(levels.size(), 3u);
  EXPECT_EQ(levels[0].size(), 1u);
  EXPECT_EQ(levels[0][0]->action.action->get_name(), "move");
  EXPECT_EQ(levels[1].size(), 1u);
  EXPECT_EQ(levels[1][0]->action.action->get_name(), "pick");
  EXPECT_EQ(levels[2].size(), 1u);
  EXPECT_EQ(levels[2][0]->action.action->get_name(), "drop");
}

// ==================== Graph Node Structure Tests ====================
TEST_F(PlanningGraphBuilderTest, NodeHasCorrectTimingInfo) {
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  auto root = graph->roots.front();
  EXPECT_FLOAT_EQ(root->action.time, 0.0f);
  EXPECT_FLOAT_EQ(root->action.duration, 10.0f);
  EXPECT_EQ(root->node_num, 0);
  EXPECT_EQ(root->level_num, 0);
}

TEST_F(PlanningGraphBuilderTest, NodeHasCorrectParams) {
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  auto root = graph->roots.front();
  ASSERT_EQ(root->action.params.size(), 3u);
  EXPECT_EQ(root->action.params[0], "robot1");
  EXPECT_EQ(root->action.params[1], "room1");
  EXPECT_EQ(root->action.params[2], "room2");
}

TEST_F(PlanningGraphBuilderTest, DependentNodeHasIncomingArc) {
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  // pick node should have move as incoming arc
  auto move_node = graph->roots.front();
  ASSERT_EQ(move_node->out_arcs.size(), 1u);

  auto pick_node = move_node->out_arcs.front();
  ASSERT_EQ(pick_node->in_arcs.size(), 1u);
  EXPECT_EQ(pick_node->in_arcs.front(), move_node);
}

// ==================== Graph Builder: Diamond Dependency ====================
TEST_F(PlanningGraphBuilderTest, DiamondDependency) {
  // Both robots move to room3 (parallel), then a single action depends on both
  // A pick at room3 requires robot1 at room3 (satisfied by first move)
  // But the second move also goes to room3
  initial_predicates_.insert(Predicate("item_at", {"item1", "room3"}, false));

  auto move_action2 = std::make_shared<MockGraphAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"robot", "robot"}, {"r1", "room"}, {"r2", "room"}});
  move_action2->add_condition(START, "robot_at", {"robot", "r1"});
  move_action2->add_condition(OVER_ALL, "battery_full", {"robot"});
  move_action2->add_effect(START, "robot_at", {"robot", "r1"}, true);
  move_action2->add_effect(END, "robot_at", {"robot", "r2"});

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room3"}, 0.0f);
  plan.add_action(move_action2, {"robot2", "room2", "room3"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room3"}, 10.001f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  // Two parallel roots
  EXPECT_EQ(graph->roots.size(), 2u);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  EXPECT_GE(levels.size(), 2u);
}

// ==================== Regression: orphan actions ====================
TEST_F(PlanningGraphBuilderTest, ActionExecutableFromInitialStateIsNotDropped) {
  // move robot1 room1->room2 (t=0), pick item1 at room2 (t=10, depends on
  // move), and an independent pick by robot2 at t=20 whose conditions already
  // hold in the initial state. The old root scan stopped at the first
  // non-executable action and never revisited the last pick, which then had
  // no parent and was dropped from execution.
  initial_predicates_.insert(Predicate("item_at", {"item2", "room2"}, false));

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);
  plan.add_action(pick_action_, {"robot2", "item2", "room2"}, 20.002f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  // move and the second pick start immediately; the first pick depends on move
  EXPECT_EQ(graph->roots.size(), 2u);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  size_t total_nodes = 0;
  std::set<std::string> reachable;
  for (const auto &level : levels) {
    total_nodes += level.size();
    for (const auto &node : level) {
      reachable.insert(node->action.action->get_name() + ":" +
                       std::to_string(node->action.time));
    }
  }
  EXPECT_EQ(total_nodes, 3u);
  EXPECT_EQ(reachable.size(), 3u);
}

TEST_F(PlanningGraphBuilderTest, EveryNonRootNodeHasAnIncomingArc) {
  // No node may be unreachable from the roots: that would silently drop an
  // action during dispatch (and can deadlock the parallel dispatcher).
  initial_predicates_.insert(Predicate("item_at", {"item2", "room2"}, false));

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);
  plan.add_action(pick_action_, {"robot2", "item2", "room2"}, 20.002f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  size_t total_nodes = 0;
  for (const auto &level : levels) {
    for (const auto &node : level) {
      total_nodes++;
      if (std::find(graph->roots.begin(), graph->roots.end(), node) ==
          graph->roots.end()) {
        EXPECT_FALSE(node->in_arcs.empty())
            << "non-root node with no incoming arc: "
            << node->action.action->get_name();
      }
    }
  }
  EXPECT_EQ(total_nodes, 3u);
}

// ==================== Regression: negated conditions ====================
TEST_F(PlanningGraphBuilderTest, NegatedConditionIsLinkedToDeletingAction) {
  auto consume_action = std::make_shared<MockGraphAction>(
      "consume",
      std::vector<std::pair<std::string, std::string>>{{"robot", "robot"}});
  consume_action->add_condition(START, "battery_full", {"robot"});
  consume_action->add_effect(START, "battery_full", {"robot"}, true);

  auto observe_action = std::make_shared<MockGraphAction>(
      "observe",
      std::vector<std::pair<std::string, std::string>>{{"robot", "robot"}});
  observe_action->add_condition(START, "battery_full", {"robot"}, true);

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(consume_action, {"robot1"}, 0.0f);
  plan.add_action(observe_action, {"robot1"}, 10.0f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  ASSERT_EQ(graph->roots.size(), 1u);
  EXPECT_EQ(graph->roots.front()->action.action->get_name(), "consume");

  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  ASSERT_EQ(levels.size(), 2u);
  ASSERT_EQ(levels[1].size(), 1u);
  EXPECT_EQ(levels[1][0]->action.action->get_name(), "observe");
  ASSERT_EQ(levels[1][0]->in_arcs.size(), 1u);
  EXPECT_EQ(levels[1][0]->in_arcs.front()->action.action->get_name(),
            "consume");
}

// ==================== Regression: end-effect mutex ====================
TEST_F(PlanningGraphBuilderTest, EndEffectConflictCreatesOrderingEdge) {
  auto finish_action = std::make_shared<MockGraphAction>(
      "finish",
      std::vector<std::pair<std::string, std::string>>{{"robot", "robot"}});
  finish_action->add_effect(END, "done", {"robot"});

  auto require_not_done = std::make_shared<MockGraphAction>(
      "require_not_done",
      std::vector<std::pair<std::string, std::string>>{{"robot", "robot"}});
  require_not_done->add_condition(START, "done", {"robot"}, true);

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(finish_action, {"robot1"}, 0.0f);
  plan.add_action(require_not_done, {"robot1"}, 1.0f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);

  // The end effect of `finish` conflicts with the negated condition of
  // `require_not_done`, so they must not run in parallel.
  auto levels = PlanningGraphBuilder::get_execution_levels(graph);
  size_t total_nodes = 0;
  bool found_dependency = false;
  for (const auto &level : levels) {
    for (const auto &node : level) {
      total_nodes++;
      if (node->action.action->get_name() == "require_not_done") {
        found_dependency =
            std::find_if(node->in_arcs.begin(), node->in_arcs.end(),
                         [](const GraphNode::Ptr &parent) {
                           return parent->action.action->get_name() == "finish";
                         }) != node->in_arcs.end();
      }
    }
  }
  EXPECT_EQ(total_nodes, 2u);
  EXPECT_TRUE(found_dependency);
}

// ==================== Regression: node reference cycles ====================
TEST_F(PlanningGraphBuilderTest, DestroyingGraphReleasesItsNodes) {
  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);
  ASSERT_FALSE(graph->roots.empty());

  GraphNode::Ptr kept = graph->roots.front();
  EXPECT_GT(kept.use_count(), 1L);

  graph.reset();

  // The graph destructor must break the in_arcs/out_arcs reference cycles,
  // leaving only the external reference.
  EXPECT_EQ(kept.use_count(), 1L);
  EXPECT_TRUE(kept->in_arcs.empty());
  EXPECT_TRUE(kept->out_arcs.empty());
}

// ==================== Planner Time Parsing Tests ====================
class PlannerTimeParsingTest : public ::testing::Test {};

TEST_F(PlannerTimeParsingTest, PlanActionStartTime) {
  Plan plan;
  plan.set_has_solution(true);
  auto action = std::make_shared<MockGraphAction>(
      "test", std::vector<std::pair<std::string, std::string>>{});

  plan.add_action(action, {}, 1.5f);
  plan.add_action(action, {}, 11.5f);
  plan.add_action(action, {}, 0.0f);

  EXPECT_FLOAT_EQ(plan.get_action_start_time(0), 1.5f);
  EXPECT_FLOAT_EQ(plan.get_action_start_time(1), 11.5f);
  EXPECT_FLOAT_EQ(plan.get_action_start_time(2), 0.0f);
}

// ==================== Execution Levels Tests ====================
TEST_F(PlanningGraphBuilderTest, ExecutionLevelsPreservesOrder) {
  // Three sequential actions
  initial_predicates_.insert(Predicate("item_at", {"item1", "room2"}, false));

  Plan plan;
  plan.set_has_solution(true);
  plan.add_action(move_action_, {"robot1", "room1", "room2"}, 0.0f);
  plan.add_action(pick_action_, {"robot1", "item1", "room2"}, 10.001f);
  plan.add_action(drop_action_, {"robot1", "item1", "room2"}, 15.002f);

  PlanningGraphBuilder builder(initial_predicates_);
  auto graph = builder.build_graph(plan);
  auto levels = PlanningGraphBuilder::get_execution_levels(graph);

  ASSERT_EQ(levels.size(), 3u);
  // Actions must be in correct order
  EXPECT_EQ(levels[0][0]->action.action->get_name(), "move");
  EXPECT_EQ(levels[1][0]->action.action->get_name(), "pick");
  EXPECT_EQ(levels[2][0]->action.action->get_name(), "drop");
}

// ==================== GraphNode Tests ====================
TEST(GraphNodeTest, MakeShared) {
  auto node = GraphNode::make_shared();
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->node_num, 0);
  EXPECT_EQ(node->level_num, 0);
  EXPECT_EQ(node->in_arcs.size(), 0u);
  EXPECT_EQ(node->out_arcs.size(), 0u);
}

TEST(PlanningGraphTest, MakeShared) {
  auto graph = PlanningGraph::make_shared();
  ASSERT_NE(graph, nullptr);
  EXPECT_EQ(graph->roots.size(), 0u);
  EXPECT_EQ(graph->levels.size(), 0u);
}

TEST(GraphNodeTest, ArcConnections) {
  auto node1 = GraphNode::make_shared();
  auto node2 = GraphNode::make_shared();

  node1->out_arcs.push_back(node2);
  node2->in_arcs.push_back(node1);

  EXPECT_EQ(node1->out_arcs.size(), 1u);
  EXPECT_EQ(node2->in_arcs.size(), 1u);
  EXPECT_EQ(node1->out_arcs.front(), node2);
  EXPECT_EQ(node2->in_arcs.front(), node1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
