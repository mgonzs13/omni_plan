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
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/planner.hpp"
#include "rclcpp/rclcpp.hpp"

#include "omni_plan_mrta/allocators/cbba_allocator.hpp"
#include "omni_plan_mrta/allocators/coalition_formation_allocator.hpp"
#include "omni_plan_mrta/allocators/greedy_auction_allocator.hpp"
#include "omni_plan_mrta/allocators/round_robin_allocator.hpp"
#include "omni_plan_mrta/allocators/ssi_affinity_allocator.hpp"
#include "omni_plan_mrta/mrta_planner.hpp"
#include "omni_plan_mrta/task_allocator.hpp"

using namespace omni_plan_mrta;
using namespace omni_plan::pddl;
using ActionMap =
    std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>;

// =============================================================================
// Test helpers
// =============================================================================
/// Builds a Problem with the given robot objects and goal predicates.
/// @p robot_objects: pairs (name, type) for robot objects
/// @p other_objects: pairs (name, type) for non-robot objects
/// @p facts: initial-state predicates
/// @p goal_predicates: goal predicates
static Problem
make_problem(const std::vector<std::pair<std::string, std::string>> &robots,
             const std::vector<std::pair<std::string, std::string>> &objects,
             const std::vector<Predicate> &facts,
             const std::vector<Predicate> &goal_preds) {
  Problem p;
  for (const auto &[n, t] : robots) {
    p.add_object(Object(n, t));
  }
  for (const auto &[n, t] : objects) {
    p.add_object(Object(n, t));
  }
  for (const auto &f : facts) {
    p.add_fact(f);
  }
  for (const auto &g : goal_preds) {
    p.add_goal(g);
  }
  return p;
}

/// Returns total number of goals assigned across all teams.
static int total_assigned(const std::vector<TeamAllocation> &teams) {
  int count = 0;
  for (const auto &t : teams) {
    count += static_cast<int>(t.goal_indices.size());
  }
  return count;
}

/// Returns the maximum number of goals assigned to any single team entry.
static int max_load(const std::vector<TeamAllocation> &teams) {
  int mx = 0;
  for (const auto &t : teams) {
    mx = std::max(mx, static_cast<int>(t.goal_indices.size()));
  }
  return mx;
}

/// Returns the minimum non-zero load across team entries that have goals.
static int min_load(const std::vector<TeamAllocation> &teams) {
  int mn = std::numeric_limits<int>::max();
  for (const auto &t : teams) {
    if (!t.goal_indices.empty()) {
      mn = std::min(mn, static_cast<int>(t.goal_indices.size()));
    }
  }
  return mn == std::numeric_limits<int>::max() ? 0 : mn;
}

/// Verifies that every goal index in [0, num_goals) appears exactly once.
static bool goals_cover(const std::vector<TeamAllocation> &teams,
                        int num_goals) {
  std::vector<int> counts(static_cast<size_t>(num_goals), 0);
  for (const auto &t : teams) {
    for (int idx : t.goal_indices) {
      if (idx < 0 || idx >= num_goals) {
        return false;
      }
      ++counts[static_cast<size_t>(idx)];
    }
  }
  for (int count : counts) {
    if (count != 1) {
      return false;
    }
  }
  return true;
}

/// Returns the first robot name in the team that owns goal_idx, or "" if none.
static std::string robot_for_goal(const std::vector<TeamAllocation> &teams,
                                  int goal_idx) {
  for (const auto &t : teams) {
    for (int idx : t.goal_indices) {
      if (idx == goal_idx) {
        return t.robots.empty() ? "" : t.robots[0];
      }
    }
  }
  return "";
}

/// Returns goal indices in the team entry that contains @p robot.
static std::vector<int>
goals_for_robot(const std::vector<TeamAllocation> &teams,
                const std::string &robot) {
  for (const auto &t : teams) {
    for (const auto &r : t.robots) {
      if (r == robot) {
        return t.goal_indices;
      }
    }
  }
  return {};
}

/// Returns true if there is a team entry whose robot list exactly equals the
/// sorted contents of @p robot_names.
static bool has_team_containing(const std::vector<TeamAllocation> &teams,
                                std::vector<std::string> robot_names) {
  std::sort(robot_names.begin(), robot_names.end());
  for (const auto &t : teams) {
    if (t.robots.size() != robot_names.size()) {
      continue;
    }
    std::vector<std::string> sorted_team = t.robots;
    std::sort(sorted_team.begin(), sorted_team.end());
    if (sorted_team == robot_names) {
      return true;
    }
  }
  return false;
}

/// Minimal concrete subclass so Action (which has pure virtuals run/cancel)
/// can be instantiated in unit tests.
class TestAction : public omni_plan::pddl::Action {
public:
  using Action::Action;
  omni_plan::pddl::ActionStatus run(const std::vector<std::string> &) override {
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  }
  void cancel() override {}
};

/// Build an ActionMap entry for a simple action with the given parameters,
/// preconditions, and effects (all at START timing, non-negated).
static std::shared_ptr<omni_plan::pddl::Action> make_test_action(
    const std::string &name,
    const std::vector<std::pair<std::string, std::string>> &params,
    const std::vector<std::pair<std::string, std::vector<std::string>>>
        &conditions,
    const std::vector<std::pair<std::string, std::vector<std::string>>>
        &effects) {
  auto a = std::make_shared<TestAction>(name, params);
  for (const auto &[pred, args] : conditions) {
    a->add_condition(omni_plan::pddl::Type::START, pred, args);
  }
  for (const auto &[pred, args] : effects) {
    a->add_effect(omni_plan::pddl::Type::END, pred, args);
  }
  return a;
}

// =============================================================================
// Common problem fixtures
// =============================================================================
// Two robots at distinct locations; two goals involving those locations.
// robot0 is at loc_a, robot1 is at loc_b.
// goal0 involves loc_a, goal1 involves loc_b.
// Any cost-aware allocator should prefer robot0→goal0, robot1→goal1.
static Problem make_two_robot_proximity_problem() {
  // robots
  std::vector<std::pair<std::string, std::string>> robots = {
      {"robot0", "robot"}, {"robot1", "robot"}};
  // non-robot objects: locations and items
  std::vector<std::pair<std::string, std::string>> objects = {
      {"loc_a", "location"},
      {"loc_b", "location"},
      {"item_x", "item"},
      {"item_y", "item"}};
  // Initial state: robot0 at loc_a, robot1 at loc_b
  // item_x at loc_a, item_y at loc_b
  std::vector<Predicate> facts = {
      Predicate("at", {"robot0", "loc_a"}),
      Predicate("at", {"robot1", "loc_b"}),
      Predicate("at", {"item_x", "loc_a"}),
      Predicate("at", {"item_y", "loc_b"}),
  };
  // goal0: pick item_x (at loc_a), goal1: pick item_y (at loc_b)
  std::vector<Predicate> goals = {
      Predicate("holding", {"robot0", "item_x"}),
      Predicate("holding", {"robot1", "item_y"}),
  };
  return make_problem(robots, objects, facts, goals);
}

// Four robots, four goals: purely structural problem for balance testing.
static Problem make_four_robot_four_goal_problem() {
  std::vector<std::pair<std::string, std::string>> robots = {
      {"r0", "robot"}, {"r1", "robot"}, {"r2", "robot"}, {"r3", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"o0", "obj"}, {"o1", "obj"}, {"o2", "obj"}, {"o3", "obj"}};
  std::vector<Predicate> goals = {
      Predicate("done", {"o0"}),
      Predicate("done", {"o1"}),
      Predicate("done", {"o2"}),
      Predicate("done", {"o3"}),
  };
  return make_problem(robots, objects, {}, goals);
}

// =============================================================================
// RoundRobinAllocator tests
// =============================================================================
class RoundRobinTest : public ::testing::Test {
protected:
  RoundRobinAllocator alloc;
  ActionMap empty_actions;
};

TEST_F(RoundRobinTest, EmptyRobotsAndGoals) {
  Problem p;
  auto result = alloc.allocate({}, {}, p, empty_actions);
  EXPECT_TRUE(result.empty());
}

TEST_F(RoundRobinTest, NoGoals) {
  Problem p;
  auto result = alloc.allocate({"r0", "r1"}, {}, p, empty_actions);
  // No goals → no allocations
  EXPECT_TRUE(result.empty());
}

TEST_F(RoundRobinTest, SingleRobotGetsAllGoals) {
  Problem p;
  std::vector<Predicate> goals = {Predicate("g", {"a"}), Predicate("g", {"b"}),
                                  Predicate("g", {"c"})};
  auto result = alloc.allocate({"r0"}, goals, p, empty_actions);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].robots[0], "r0");
  EXPECT_EQ(result[0].goal_indices.size(), 3u);
  EXPECT_TRUE(goals_cover(result, 3));
}

TEST_F(RoundRobinTest, TwoRobotsEvenGoals) {
  auto result = alloc.allocate({"r0", "r1"},
                               {Predicate("g", {"a"}), Predicate("g", {"b"}),
                                Predicate("g", {"c"}), Predicate("g", {"d"})},
                               Problem{}, empty_actions);

  EXPECT_TRUE(goals_cover(result, 4));
  const auto &r0 = goals_for_robot(result, "r0");
  const auto &r1 = goals_for_robot(result, "r1");
  EXPECT_EQ(r0.size(), 2u);
  EXPECT_EQ(r1.size(), 2u);
  // Round-robin: r0 gets 0,2 — r1 gets 1,3
  EXPECT_NE(std::find(r0.begin(), r0.end(), 0), r0.end());
  EXPECT_NE(std::find(r0.begin(), r0.end(), 2), r0.end());
  EXPECT_NE(std::find(r1.begin(), r1.end(), 1), r1.end());
  EXPECT_NE(std::find(r1.begin(), r1.end(), 3), r1.end());
}

TEST_F(RoundRobinTest, TwoRobotsOddGoals) {
  auto result = alloc.allocate(
      {"r0", "r1"},
      {Predicate("g", {"a"}), Predicate("g", {"b"}), Predicate("g", {"c"})},
      Problem{}, empty_actions);

  EXPECT_TRUE(goals_cover(result, 3));
  EXPECT_EQ(total_assigned(result), 3);
  // One robot has 2, the other 1 — difference at most 1
  EXPECT_LE(max_load(result) - min_load(result), 1);
}

TEST_F(RoundRobinTest, FourRobotsFourGoalsPerfectBalance) {
  auto p = make_four_robot_four_goal_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result =
      alloc.allocate({"r0", "r1", "r2", "r3"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 4));
  EXPECT_EQ(max_load(result), 1);
  EXPECT_EQ(min_load(result), 1);
}

TEST_F(RoundRobinTest, AllGoalsAssigned) {
  auto p = make_two_robot_proximity_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result = alloc.allocate({"robot0", "robot1"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, static_cast<int>(goals.size())));
  EXPECT_EQ(total_assigned(result), static_cast<int>(goals.size()));
}

// =============================================================================
// SsiAffinityAllocator tests
// =============================================================================
class SsiAffinityTest : public ::testing::Test {
protected:
  SsiAffinityAllocator alloc;
  ActionMap empty_actions;
};

TEST_F(SsiAffinityTest, EmptyRobotsAndGoals) {
  Problem p;
  auto result = alloc.allocate({}, {}, p, empty_actions);
  EXPECT_TRUE(result.empty());
}

TEST_F(SsiAffinityTest, NoInitialFacts_FallsBackToLoadBalance) {
  // No facts means zero affinity for all pairs — load balancing takes over.
  std::vector<Predicate> goals = {Predicate("g", {"a"}), Predicate("g", {"b"})};
  auto result = alloc.allocate({"r0", "r1"}, goals, Problem{}, empty_actions);

  EXPECT_TRUE(goals_cover(result, 2));
  EXPECT_EQ(total_assigned(result), 2);
  // With zero affinity, load balancing should distribute 1 each
  EXPECT_EQ(max_load(result), 1);
  EXPECT_EQ(min_load(result), 1);
}

TEST_F(SsiAffinityTest, AffinityDrivenAssignment) {
  // In this problem the 1-hop affinity is zero for all (robot, goal) pairs
  // because no single S0 fact mentions both a robot AND a goal argument.
  // The allocator must still assign all goals (falls back to load balancing).
  auto p = make_two_robot_proximity_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result = alloc.allocate({"robot0", "robot1"}, goals, p, empty_actions);
  EXPECT_TRUE(goals_cover(result, static_cast<int>(goals.size())));
  EXPECT_EQ(total_assigned(result), static_cast<int>(goals.size()));
}

TEST_F(SsiAffinityTest, DirectCoMentionAffinity) {
  // Build a problem where robot0 co-mentions item_a in the same fact
  // and robot1 co-mentions item_b in the same fact.
  std::vector<std::pair<std::string, std::string>> robots = {{"r0", "robot"},
                                                             {"r1", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"item_a", "item"}, {"item_b", "item"}};
  // "carries" fact puts robot and item in the same predicate
  std::vector<Predicate> facts = {Predicate("carries", {"r0", "item_a"}),
                                  Predicate("carries", {"r1", "item_b"})};
  // goal0 involves item_a (affinity(r0, g0)=1, affinity(r1, g0)=0)
  // goal1 involves item_b (affinity(r0, g1)=0, affinity(r1, g1)=1)
  std::vector<Predicate> goal_preds = {Predicate("done", {"item_a"}),
                                       Predicate("done", {"item_b"})};
  auto p = make_problem(robots, objects, facts, goal_preds);
  std::vector<Predicate> goals(goal_preds.begin(), goal_preds.end());

  auto result = alloc.allocate({"r0", "r1"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 2));
  // r0 should get goal0 (involves item_a) and r1 should get goal1 (involves
  // item_b)
  EXPECT_EQ(robot_for_goal(result, 0), "r0");
  EXPECT_EQ(robot_for_goal(result, 1), "r1");
}

TEST_F(SsiAffinityTest, LoadBalancingWithEqualAffinity) {
  // All robots have equal affinity to all goals → must still distribute evenly
  std::vector<Predicate> goals;
  for (int i = 0; i < 6; ++i) {
    goals.emplace_back(Predicate("g", {std::to_string(i)}));
  }
  auto result =
      alloc.allocate({"r0", "r1", "r2"}, goals, Problem{}, empty_actions);

  EXPECT_TRUE(goals_cover(result, 6));
  // Perfect balance: 2 goals each
  EXPECT_EQ(max_load(result), 2);
  EXPECT_EQ(min_load(result), 2);
}

// =============================================================================
// GreedyAuctionAllocator tests
// =============================================================================
class GreedyAuctionTest : public ::testing::Test {
protected:
  GreedyAuctionAllocator alloc;
  ActionMap empty_actions;
};

TEST_F(GreedyAuctionTest, EmptyRobotsAndGoals) {
  Problem p;
  auto result = alloc.allocate({}, {}, p, empty_actions);
  EXPECT_TRUE(result.empty());
}

TEST_F(GreedyAuctionTest, AllGoalsAssigned) {
  auto p = make_two_robot_proximity_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result = alloc.allocate({"robot0", "robot1"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, static_cast<int>(goals.size())));
  EXPECT_EQ(total_assigned(result), static_cast<int>(goals.size()));
}

TEST_F(GreedyAuctionTest, BfsProximityAssignment) {
  // BFS graph: robot0 -- loc_a -- item_x
  //            robot1 -- loc_b -- item_y
  // Distance(robot0, goal needing item_x) = 2 (robot0→loc_a→item_x)
  // Distance(robot1, goal needing item_x) = ≥3 (robot1→loc_b→... no direct
  // path) So robot0 should prefer item_x goals, robot1 should prefer item_y
  // goals.

  std::vector<std::pair<std::string, std::string>> robots = {{"r0", "robot"},
                                                             {"r1", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"loc_a", "location"},
      {"loc_b", "location"},
      {"item_x", "item"},
      {"item_y", "item"}};
  std::vector<Predicate> facts = {Predicate("at", {"r0", "loc_a"}),
                                  Predicate("at", {"r1", "loc_b"}),
                                  Predicate("at", {"item_x", "loc_a"}),
                                  Predicate("at", {"item_y", "loc_b"})};
  // goal0 = done(item_x), goal1 = done(item_y)
  std::vector<Predicate> goal_preds = {Predicate("done", {"item_x"}),
                                       Predicate("done", {"item_y"})};

  auto p = make_problem(robots, objects, facts, goal_preds);
  auto result = alloc.allocate({"r0", "r1"}, goal_preds, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 2));
  // r0 is closer to item_x, r1 is closer to item_y
  EXPECT_EQ(robot_for_goal(result, 0), "r0");
  EXPECT_EQ(robot_for_goal(result, 1), "r1");
}

TEST_F(GreedyAuctionTest, LoadBalancingNoFacts) {
  // No initial state → all BFS distances equal → load balancing kicks in
  std::vector<Predicate> goals;
  for (int i = 0; i < 6; ++i) {
    goals.emplace_back(Predicate("g", {std::to_string(i)}));
  }
  auto result =
      alloc.allocate({"r0", "r1", "r2"}, goals, Problem{}, empty_actions);

  EXPECT_TRUE(goals_cover(result, 6));
  EXPECT_EQ(max_load(result), 2);
  EXPECT_EQ(min_load(result), 2);
}

TEST_F(GreedyAuctionTest, FourRobotsFourGoalsCovered) {
  auto p = make_four_robot_four_goal_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result =
      alloc.allocate({"r0", "r1", "r2", "r3"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 4));
  EXPECT_EQ(total_assigned(result), 4);
}

TEST_F(GreedyAuctionTest, DistantReachableGoalBeatsUnreachablePair) {
  // A chain n0..n10 gives robot0 a finite BFS distance of 11 to goal0, which
  // is larger than the old sentinel N*M+1 = 3. robot1 cannot reach goal0 at
  // all, so it must not outscore robot0's distant-but-reachable assignment.
  std::vector<std::pair<std::string, std::string>> robots = {{"r0", "robot"},
                                                             {"r1", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"m0", "place"}};
  std::vector<Predicate> facts = {Predicate("at", {"r0", "n0"}),
                                  Predicate("at", {"r1", "m0"})};
  for (int i = 0; i < 10; ++i) {
    facts.push_back(Predicate("link", {"n" + std::to_string(i),
                                       "n" + std::to_string(i + 1)}));
  }
  std::vector<Predicate> goal_preds = {Predicate("done", {"n10"})};
  auto p = make_problem(robots, objects, facts, goal_preds);

  auto result = alloc.allocate({"r0", "r1"}, goal_preds, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 1));
  EXPECT_EQ(robot_for_goal(result, 0), "r0");
}

// =============================================================================
// CbbaAllocator tests
// =============================================================================
class CbbaTest : public ::testing::Test {
protected:
  ActionMap empty_actions;
};

TEST_F(CbbaTest, EmptyRobotsAndGoals) {
  CbbaAllocator alloc;
  Problem p;
  auto result = alloc.allocate({}, {}, p, empty_actions);
  EXPECT_TRUE(result.empty());
}

TEST_F(CbbaTest, AllGoalsAssignedHAdd) {
  CbbaAllocator alloc(/*use_h_max=*/false);
  auto p = make_two_robot_proximity_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result = alloc.allocate({"robot0", "robot1"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, static_cast<int>(goals.size())));
  EXPECT_EQ(total_assigned(result), static_cast<int>(goals.size()));
}

TEST_F(CbbaTest, AllGoalsAssignedHMax) {
  CbbaAllocator alloc(/*use_h_max=*/true);
  auto p = make_two_robot_proximity_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result = alloc.allocate({"robot0", "robot1"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, static_cast<int>(goals.size())));
  EXPECT_EQ(total_assigned(result), static_cast<int>(goals.size()));
}

TEST_F(CbbaTest, NoActionsNoGoalCostFallbackToLoadBalance) {
  // With no actions, all heuristic costs are "unreachable" (infinity).
  // CBBA assigns every goal to the first robot (all bids are equal) before
  // the load-balancing penalty kicks in during the bundle phase.
  // The important property is that every goal is assigned exactly once.
  CbbaAllocator alloc;
  std::vector<Predicate> goals;
  for (int i = 0; i < 4; ++i) {
    goals.emplace_back(Predicate("g", {std::to_string(i)}));
  }
  auto result = alloc.allocate({"r0", "r1"}, goals, Problem{}, empty_actions);

  EXPECT_TRUE(goals_cover(result, 4));
  EXPECT_EQ(total_assigned(result), 4);
}

// Note: omni_plan::pddl::Action is abstract (has pure virtual run()/cancel()),
// so it cannot be instantiated directly in tests. The allocators are exercised
// with empty action maps instead; they fall back to load balancing in that
// case.

TEST_F(CbbaTest, FourRobotsFourGoalsCovered) {
  CbbaAllocator alloc;
  auto p = make_four_robot_four_goal_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result =
      alloc.allocate({"r0", "r1", "r2", "r3"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 4));
  EXPECT_EQ(total_assigned(result), 4);
}

TEST_F(CbbaTest, ConstructorValueSurvivesParameterLoad) {
  // The registered ROS default must be the constructor value, otherwise
  // loading parameters from a node that does not define allocator.use_h_max
  // silently reverts use_h_max to false. The declared default is a public
  // observable of load_ros_parameters().
  auto node = std::make_shared<rclcpp::Node>("cbba_ctor_value_test");
  CbbaAllocator alloc(/*use_h_max=*/true);
  alloc.set_namespace("cbba_ctor_value_test");
  alloc.load_ros_parameters(node);

  EXPECT_TRUE(node->get_parameter("cbba_ctor_value_test.use_h_max").as_bool());
}

TEST_F(CbbaTest, HMaxConstructorValueUsedAfterParameterLoad) {
  // Behavioral check for the constructor default: with h_max, robot r0
  // reaches done(obj) through two parallel one-step facts (h_max = 2 vs
  // h_add = 3), while r1's chain costs 3 under both. h_max therefore gives
  // r0 a strictly better bid; h_add ties and keeps the lower-index robot r1.
  auto node = std::make_shared<rclcpp::Node>("cbba_ctor_hmax_behavior");
  CbbaAllocator alloc(/*use_h_max=*/true);
  alloc.set_namespace("cbba_ctor_hmax_behavior");
  alloc.load_ros_parameters(node);

  std::vector<std::pair<std::string, std::string>> robots = {{"r0", "robot"},
                                                             {"r1", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"obj", "object"}};
  std::vector<Predicate> facts = {Predicate("p", {"r0", "obj"}),
                                  Predicate("q", {"r0", "obj"}),
                                  Predicate("s", {"r1", "obj"})};
  std::vector<Predicate> goals = {Predicate("done", {"obj"})};
  auto p = make_problem(robots, objects, facts, goals);

  ActionMap actions;
  actions["mkp"] = make_test_action("mkp", {{"?r", "robot"}, {"?o", "object"}},
                                    {{"p", {"?r", "?o"}}}, {{"mp", {"?r", "?o"}}});
  actions["mkq"] = make_test_action("mkq", {{"?r", "robot"}, {"?o", "object"}},
                                    {{"q", {"?r", "?o"}}}, {{"mq", {"?r", "?o"}}});
  actions["finish"] =
      make_test_action("finish", {{"?r", "robot"}, {"?o", "object"}},
                       {{"mp", {"?r", "?o"}}, {"mq", {"?r", "?o"}}},
                       {{"done", {"?o"}}});
  actions["step1"] =
      make_test_action("step1", {{"?r", "robot"}, {"?o", "object"}},
                       {{"s", {"?r", "?o"}}}, {{"t", {"?r", "?o"}}});
  actions["step2"] =
      make_test_action("step2", {{"?r", "robot"}, {"?o", "object"}},
                       {{"t", {"?r", "?o"}}}, {{"u", {"?r", "?o"}}});
  actions["solo"] =
      make_test_action("solo", {{"?r", "robot"}, {"?o", "object"}},
                       {{"u", {"?r", "?o"}}}, {{"done", {"?o"}}});

  auto result = alloc.allocate({"r1", "r0"}, goals, p, actions);

  EXPECT_TRUE(goals_cover(result, 1));
  EXPECT_EQ(robot_for_goal(result, 0), "r0");
}

TEST_F(CbbaTest, HugeHAddCostDoesNotOverflowBids) {
  // r0 reaches done(obj) only through a 19-level chain whose h_add cost is
  // ~5.8e8 (still finite). r1 reaches it in one step. The bid computation
  // h_cost * dist_scale + bfs overflows int without saturation, flipping r0's
  // bid positive and wrongly awarding it the goal.
  std::vector<std::pair<std::string, std::string>> robots = {{"r0", "robot"},
                                                             {"r1", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"obj", "object"}};
  std::vector<Predicate> facts = {Predicate("at", {"r0", "seed"}),
                                  Predicate("has", {"r1", "obj"}),
                                  Predicate("edge", {"seed", "n1"}),
                                  Predicate("edge", {"n1", "n2"}),
                                  Predicate("edge", {"n2", "n3"}),
                                  Predicate("edge", {"n3", "obj"})};
  std::vector<Predicate> goals = {Predicate("done", {"obj"})};
  auto p = make_problem(robots, objects, facts, goals);

  ActionMap actions;
  actions["level1"] = make_test_action("level1", {{"?r", "robot"}},
                                       {{"at", {"?r", "seed"}}},
                                       {{"a1", {"?r"}}});
  const int kLevels = 19;
  for (int level = 2; level <= kLevels; ++level) {
    std::vector<std::pair<std::string, std::vector<std::string>>> conds;
    for (int k = 0; k < 3; ++k) {
      conds.push_back({"a" + std::to_string(level - 1), {"?r"}});
    }
    actions["level" + std::to_string(level)] =
        make_test_action("level" + std::to_string(level), {{"?r", "robot"}},
                         conds, {{"a" + std::to_string(level), {"?r"}}});
  }
  actions["finish"] = make_test_action("finish",
                                       {{"?r", "robot"}, {"?o", "object"}},
                                       {{"a19", {"?r"}}}, {{"done", {"?o"}}});
  actions["cheap"] =
      make_test_action("cheap", {{"?r", "robot"}, {"?o", "object"}},
                       {{"has", {"?r", "?o"}}}, {{"done", {"?o"}}});

  CbbaAllocator alloc(/*use_h_max=*/false);
  auto result = alloc.allocate({"r0", "r1"}, goals, p, actions);

  EXPECT_TRUE(goals_cover(result, 1));
  EXPECT_EQ(robot_for_goal(result, 0), "r1");
}

// =============================================================================
// Cross-allocator completeness tests
// =============================================================================
class AllocatorCompletenessTest
    : public ::testing::TestWithParam<TaskAllocator *> {
protected:
  ActionMap empty_actions;
};

TEST_P(AllocatorCompletenessTest, SingleGoalAlwaysAssigned) {
  TaskAllocator *alloc = GetParam();
  std::vector<Predicate> goals = {Predicate("done", {"obj0"})};
  auto result = alloc->allocate({"r0", "r1"}, goals, Problem{}, empty_actions);
  EXPECT_TRUE(goals_cover(result, 1));
}

TEST_P(AllocatorCompletenessTest, MoreGoalsThanRobots) {
  TaskAllocator *alloc = GetParam();
  std::vector<Predicate> goals;
  for (int i = 0; i < 7; ++i) {
    goals.emplace_back(Predicate("g", {std::to_string(i)}));
  }
  auto result =
      alloc->allocate({"r0", "r1", "r2"}, goals, Problem{}, empty_actions);
  EXPECT_TRUE(goals_cover(result, 7));
  EXPECT_EQ(total_assigned(result), 7);
}

TEST_P(AllocatorCompletenessTest, MoreRobotsThanGoals) {
  TaskAllocator *alloc = GetParam();
  std::vector<Predicate> goals = {Predicate("g", {"a"}), Predicate("g", {"b"})};
  auto result = alloc->allocate({"r0", "r1", "r2", "r3"}, goals, Problem{},
                                empty_actions);
  EXPECT_TRUE(goals_cover(result, 2));
  EXPECT_EQ(total_assigned(result), 2);
}

// Instantiate with one instance of each concrete allocator.
// These are owning pointers managed by the test suite lifecycle.
RoundRobinAllocator g_rr;
SsiAffinityAllocator g_ssi;
GreedyAuctionAllocator g_bfs;
CbbaAllocator g_cbba_hadd(false);
CbbaAllocator g_cbba_hmax(true);
CoalitionFormationAllocator g_coalition;

// INSTANTIATE_TEST_SUITE_P was introduced in GTest 1.10; fall back to the
// deprecated INSTANTIATE_TEST_CASE_P on older releases.
#ifdef INSTANTIATE_TEST_SUITE_P
// Named function avoids preprocessor comma-splitting inside {…} array literal.
static std::string
AllocatorTestName(const ::testing::TestParamInfo<TaskAllocator *> &info) {
  switch (info.index) {
  case 0:
    return "RoundRobin";
  case 1:
    return "SsiAffinity";
  case 2:
    return "GreedyBFS";
  case 3:
    return "CbbaHAdd";
  case 4:
    return "CbbaHMax";
  case 5:
    return "CoalitionFormation";
  default:
    return "Unknown";
  }
}

INSTANTIATE_TEST_SUITE_P(AllAllocators, AllocatorCompletenessTest,
                         ::testing::Values(&g_rr, &g_ssi, &g_bfs, &g_cbba_hadd,
                                           &g_cbba_hmax, &g_coalition),
                         AllocatorTestName);
#else
INSTANTIATE_TEST_CASE_P(AllAllocators, AllocatorCompletenessTest,
                        ::testing::Values(&g_rr, &g_ssi, &g_bfs, &g_cbba_hadd,
                                          &g_cbba_hmax, &g_coalition));
#endif

// =============================================================================
// CoalitionFormationAllocator tests
// =============================================================================

class CoalitionFormationTest : public ::testing::Test {
protected:
  ActionMap empty_actions;
};

TEST_F(CoalitionFormationTest, EmptyRobotsAndGoals) {
  CoalitionFormationAllocator alloc;
  Problem p;
  auto result = alloc.allocate({}, {}, p, empty_actions);
  EXPECT_TRUE(result.empty());
}

TEST_F(CoalitionFormationTest, AllSrGoalsDistributed) {
  // No action schemas → everything treated as SR (fallback).
  CoalitionFormationAllocator alloc;
  auto p = make_two_robot_proximity_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result = alloc.allocate({"robot0", "robot1"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, static_cast<int>(goals.size())));
  EXPECT_EQ(total_assigned(result), static_cast<int>(goals.size()));
  // All teams should be single-robot
  for (const auto &t : result) {
    EXPECT_EQ(t.robots.size(), 1u);
  }
}

TEST_F(CoalitionFormationTest, TeamGoalFormsTwoRobotCoalition) {
  // Create a problem where goal0 = "handover(r0,r1,obj1)" can only be
  // achieved by an action with two robot parameters.
  //
  // We cannot instantiate omni_plan::pddl::Action directly (abstract), but we
  // CAN create a concrete struct-like wrapper using a TestAction subclass.
  // Since Action is abstract in this test environment, we instead verify the
  // classification path indirectly: if no achiever is found (empty action map)
  // the allocator degrades gracefully to SR.  The coalition path is tested
  // through CoalitionDetectedFromActionSchema below which uses a mock.

  CoalitionFormationAllocator alloc;
  std::vector<std::pair<std::string, std::string>> robots = {{"r0", "robot"},
                                                             {"r1", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"obj1", "object"}};
  // Goal: needs two robots (by naming convention the goal arg count is 3)
  std::vector<Predicate> goals = {Predicate("handover", {"r0", "r1", "obj1"})};
  auto p = make_problem(robots, objects, {}, goals);

  // With no action schemas, goal is treated as SR fallback.
  auto result = alloc.allocate({"r0", "r1"}, goals, p, empty_actions);
  EXPECT_TRUE(goals_cover(result, 1));
  EXPECT_EQ(total_assigned(result), 1);
}

TEST_F(CoalitionFormationTest, FourRobotsAllSrCovered) {
  CoalitionFormationAllocator alloc;
  auto p = make_four_robot_four_goal_problem();
  std::vector<Predicate> goals(p.get_goals().begin(), p.get_goals().end());
  auto result =
      alloc.allocate({"r0", "r1", "r2", "r3"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 4));
  EXPECT_EQ(total_assigned(result), 4);
}

TEST_F(CoalitionFormationTest, SingleRobotGetsAllGoals) {
  CoalitionFormationAllocator alloc;
  Problem p;
  std::vector<Predicate> goals = {Predicate("g", {"a"}), Predicate("g", {"b"}),
                                  Predicate("g", {"c"})};
  auto result = alloc.allocate({"r0"}, goals, p, empty_actions);
  EXPECT_TRUE(goals_cover(result, 3));
  EXPECT_EQ(total_assigned(result), 3);
}

TEST_F(CoalitionFormationTest, ConstructorValueSurvivesParameterLoad) {
  auto node = std::make_shared<rclcpp::Node>("coalition_ctor_value_test");
  CoalitionFormationAllocator alloc(5);
  alloc.set_namespace("coalition_ctor_value_test");
  alloc.load_ros_parameters(node);

  EXPECT_EQ(
      node->get_parameter("coalition_ctor_value_test.max_coalition_size")
          .as_int(),
      5);
}

TEST_F(CoalitionFormationTest, FourRobotCoalitionSurvivesParameterLoad) {
  // A goal that needs a chain of four complementary actions must be solved by
  // the coalition {r1, r2, r3, r4}. Constructing with max_coalition_size=5 and
  // then loading parameters (without allocator.max_coalition_size set) must
  // keep 5; otherwise the cap of 3 prevents the 4-robot coalition.
  auto node = std::make_shared<rclcpp::Node>("coalition_four_team_test");
  CoalitionFormationAllocator alloc(5);
  alloc.set_namespace("coalition_four_team_test");
  alloc.load_ros_parameters(node);

  std::vector<std::pair<std::string, std::string>> robots = {
      {"r1", "robot1"}, {"r2", "robot2"}, {"r3", "robot3"}, {"r4", "robot4"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"obj", "object"}};
  std::vector<Predicate> facts = {Predicate("base", {"obj"})};
  std::vector<Predicate> goals = {Predicate("done", {"obj"})};
  auto p = make_problem(robots, objects, facts, goals);

  ActionMap actions;
  actions["s1"] = make_test_action("s1", {{"?a", "robot1"}, {"?o", "object"}},
                                   {{"base", {"?o"}}}, {{"f1", {"?o"}}});
  actions["s2"] = make_test_action("s2", {{"?a", "robot2"}, {"?o", "object"}},
                                   {{"f1", {"?o"}}}, {{"f2", {"?o"}}});
  actions["s3"] = make_test_action("s3", {{"?a", "robot3"}, {"?o", "object"}},
                                   {{"f2", {"?o"}}}, {{"f3", {"?o"}}});
  actions["s4"] = make_test_action("s4", {{"?a", "robot4"}, {"?o", "object"}},
                                   {{"f3", {"?o"}}}, {{"done", {"?o"}}});

  auto result =
      alloc.allocate({"r1", "r2", "r3", "r4"}, goals, p, actions);

  EXPECT_TRUE(goals_cover(result, 1));
  EXPECT_TRUE(has_team_containing(result, {"r1", "r2", "r3", "r4"}));
}

TEST_F(CoalitionFormationTest, HugeLoadBonusDoesNotOverflow) {
  // r_cap can achieve special(r_incap) with a finite h-cost but is far away;
  // r_incap cannot achieve it at all but its BFS distance is 0. The
  // capability bonus must stay positive so r_cap wins. The "big" goal sums
  // BFS distances along a 46300-node chain, making max_finite_dist ~1.07e9;
  // with max_finite_dist + load_coeff * M computed in int the bonus wraps
  // negative and r_incap wins special instead.
  const int kChain = 46300;
  const int kTotalGoals = 3;

  std::vector<std::pair<std::string, std::string>> robots = {
      {"r_cap", "robot"}, {"r_incap", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"obj", "object"}};
  std::vector<Predicate> facts = {
      Predicate("can_do", {"r_cap"}),
      Predicate("at", {"r_cap", "c0"}),
      Predicate("at", {"r_incap", "c" + std::to_string(kChain)})};
  for (int i = 0; i < kChain; ++i) {
    facts.push_back(Predicate("edge", {"c" + std::to_string(i),
                                       "c" + std::to_string(i + 1)}));
  }

  std::vector<std::string> big_args;
  for (int i = 1; i <= kChain; ++i) {
    big_args.push_back("c" + std::to_string(i));
  }
  std::vector<Predicate> goals;
  goals.emplace_back("special", std::vector<std::string>{"r_incap"});
  goals.emplace_back("big", big_args);
  goals.emplace_back("filler", std::vector<std::string>{"f0"});
  auto p = make_problem(robots, objects, facts, goals);

  ActionMap actions;
  actions["do"] = make_test_action("do", {{"?r", "robot"}, {"?x", "robot"}},
                                   {{"can_do", {"?r"}}},
                                   {{"special", {"?x"}}});

  CoalitionFormationAllocator alloc;
  auto result = alloc.allocate({"r_cap", "r_incap"}, goals, p, actions);

  EXPECT_TRUE(goals_cover(result, kTotalGoals));
  EXPECT_EQ(robot_for_goal(result, 0), "r_cap");
}

TEST_F(CoalitionFormationTest, TeamGoalBfsProximityCoalitionChoice) {
  // Three robots: r0 at loc_a, r1 at loc_b, r2 at loc_b.
  // Team goal = "collab(r,r,obj_b)" (requires 2 robots).
  // No action schemas → falls back to BFS/SR, still covers the goal.
  CoalitionFormationAllocator alloc;
  std::vector<std::pair<std::string, std::string>> robots = {
      {"r0", "robot"}, {"r1", "robot"}, {"r2", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"loc_a", "location"}, {"loc_b", "location"}, {"obj_b", "object"}};
  std::vector<Predicate> facts = {
      Predicate("at", {"r0", "loc_a"}), Predicate("at", {"r1", "loc_b"}),
      Predicate("at", {"r2", "loc_b"}), Predicate("at", {"obj_b", "loc_b"})};
  // Two individual goals + one "team-like" goal (without action schema, SR)
  std::vector<Predicate> goals = {
      Predicate("done", {"r0"}),      // SR: obviously for r0
      Predicate("done", {"obj_b"}),   // SR near loc_b → r1 or r2
      Predicate("shared", {"obj_b"}), // SR (no schema)
  };
  auto p = make_problem(robots, objects, facts, goals);
  auto result = alloc.allocate({"r0", "r1", "r2"}, goals, p, empty_actions);

  EXPECT_TRUE(goals_cover(result, 3));
  EXPECT_EQ(total_assigned(result), 3);
}

TEST_F(CoalitionFormationTest, ComplementarySkillsFormCoalition) {
  // Scenario: robot_a can PREPARE an object; robot_b can PROCESS it.
  // Goal: processed(obj1).
  //
  //   Action prepare:
  //     ?r - robot_pick, ?o - object
  //     pre:  available(?o)
  //     eff:  prepared(?o)
  //
  //   Action process:
  //     ?r - robot_cook, ?o - object
  //     pre:  prepared(?o)
  //     eff:  processed(?o)
  //
  // Neither robot alone can achieve processed(obj1):
  //   - robot_a (robot_pick): can prepare, but has no process action.
  //   - robot_b (robot_cook): has process, but prepared precondition is
  //     never satisfied without robot_a.
  // Together {robot_a, robot_b} can: robot_a prepares, robot_b processes.

  CoalitionFormationAllocator alloc;

  // Objects
  std::vector<std::pair<std::string, std::string>> robots = {
      {"robot_a", "robot_pick"}, {"robot_b", "robot_cook"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"obj1", "object"}};
  std::vector<Predicate> facts = {Predicate("available", {"obj1"})};
  std::vector<Predicate> goals = {Predicate("processed", {"obj1"})};

  auto p = make_problem(robots, objects, facts, goals);

  ActionMap actions;
  // prepare(?r - robot_pick, ?o - object): available(?o) → prepared(?o)
  actions["prepare"] =
      make_test_action("prepare", {{"?r", "robot_pick"}, {"?o", "object"}},
                       {{"available", {"?o"}}}, {{"prepared", {"?o"}}});
  // process(?r - robot_cook, ?o - object): prepared(?o) → processed(?o)
  actions["process"] =
      make_test_action("process", {{"?r", "robot_cook"}, {"?o", "object"}},
                       {{"prepared", {"?o"}}}, {{"processed", {"?o"}}});

  auto result = alloc.allocate({"robot_a", "robot_b"}, goals, p, actions);

  // All goals must be covered.
  EXPECT_TRUE(goals_cover(result, 1));
  EXPECT_EQ(total_assigned(result), 1);

  // The goal must be assigned to the coalition {robot_a, robot_b}.
  EXPECT_TRUE(has_team_containing(result, {"robot_a", "robot_b"}));
}

TEST_F(CoalitionFormationTest, SrGoalAssignedToCapableRobot) {
  // Scenario: only robot_cook can process (no prepare needed for a
  // different goal). robot_pick just has prepare, so processed(obj1) is
  // only achievable by robot_cook alone.
  //
  //   Action process_direct:
  //     ?r - robot_cook, ?o - object
  //     pre:  available(?o)
  //     eff:  processed(?o)
  //
  // robot_cook alone achieves the goal → SR, should be assigned to robot_cook.

  CoalitionFormationAllocator alloc;

  std::vector<std::pair<std::string, std::string>> robots = {
      {"robot_a", "robot_pick"}, {"robot_b", "robot_cook"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"obj1", "object"}};
  std::vector<Predicate> facts = {Predicate("available", {"obj1"})};
  std::vector<Predicate> goals = {Predicate("processed", {"obj1"})};

  auto p = make_problem(robots, objects, facts, goals);

  ActionMap actions;
  // process_direct(?r - robot_cook, ?o - object): available(?o) → processed(?o)
  actions["process_direct"] = make_test_action(
      "process_direct", {{"?r", "robot_cook"}, {"?o", "object"}},
      {{"available", {"?o"}}}, {{"processed", {"?o"}}});

  auto result = alloc.allocate({"robot_a", "robot_b"}, goals, p, actions);

  EXPECT_TRUE(goals_cover(result, 1));
  EXPECT_EQ(total_assigned(result), 1);

  // Must be assigned as a solo entry for robot_b (the capable one).
  const std::string owner = robot_for_goal(result, 0);
  EXPECT_EQ(owner, "robot_b");
}

// =============================================================================
// MrtaPlanner tests
// =============================================================================
namespace {

/// Canonical key for a predicate: "name(a1,a2,...)".
std::string pred_key(const Predicate &pred) {
  std::string key = pred.get_name() + "(";
  const auto &args = pred.get_args();
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0) {
      key += ',';
    }
    key += args[i];
  }
  return key + ")";
}

/// Fake allocator that returns a preconfigured allocation.
class FakeTaskAllocator : public TaskAllocator {
public:
  std::vector<TeamAllocation> allocation;

  std::vector<TeamAllocation>
  allocate(const std::vector<std::string> &,
           const std::vector<omni_plan::pddl::Predicate> &,
           const omni_plan::pddl::Problem &, const ActionMap &) const override {
    return allocation;
  }
};

/// Summary of a sub-problem received by FakeSubPlanner.
struct SubProblemRecord {
  std::vector<std::string> robots;
  std::vector<std::string> goals;
};

/// Fake sub-planner: records each sub-problem and either solves it with one
/// action or reports failure based on fail_pred_.
class FakeSubPlanner : public omni_plan::Planner {
public:
  explicit FakeSubPlanner(std::shared_ptr<omni_plan::pddl::Action> action)
      : action_(std::move(action)) {}

  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &,
                const omni_plan::pddl::Problem &problem) const override {
    SubProblemRecord record;
    for (const auto &obj : problem.get_objects()) {
      if (obj.get_type() == "robot") {
        record.robots.push_back(obj.get_name());
      }
    }
    for (const auto &goal : problem.get_goals()) {
      record.goals.push_back(pred_key(goal));
    }

    bool solve = true;
    if (fail_pred_) {
      solve = !fail_pred_(problem);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    records_.push_back(std::move(record));

    omni_plan::pddl::Plan plan;
    plan.set_has_solution(solve);
    if (solve) {
      int i = 0;
      for (const auto &goal : problem.get_goals()) {
        plan.add_action(action_, {pred_key(goal), std::to_string(i++)}, 0.0f);
      }
    }
    return plan;
  }

  omni_plan::pddl::Plan parse_plan(const omni_plan::pddl::Domain &,
                                   const std::string &) const override {
    return omni_plan::pddl::Plan{};
  }

  std::function<bool(const omni_plan::pddl::Problem &)> fail_pred_;

  std::vector<SubProblemRecord> records() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return records_;
  }

private:
  std::shared_ptr<omni_plan::pddl::Action> action_;
  mutable std::mutex mutex_;
  mutable std::vector<SubProblemRecord> records_;
};

/// True if @p record contains exactly the goal keys in @p names.
bool has_exact_goals(const SubProblemRecord &record,
                     const std::vector<std::string> &names) {
  if (record.goals.size() != names.size()) {
    return false;
  }
  for (const auto &name : names) {
    if (std::find(record.goals.begin(), record.goals.end(), name) ==
        record.goals.end()) {
      return false;
    }
  }
  return true;
}

} // namespace

/// Subclass used to inject the test doubles through the protected
/// dependency-injection seam, without any test-only symbols in the header.
class TestMrtaPlanner : public MrtaPlanner {
public:
  void inject(std::shared_ptr<rclcpp::Node> node,
              std::shared_ptr<omni_plan::Planner> sub_planner,
              std::shared_ptr<TaskAllocator> allocator,
              const std::string &robot_type = "robot") {
    this->node_ = std::move(node);
    this->sub_planner_ = std::move(sub_planner);
    this->allocator_ = std::move(allocator);
    this->robot_type_ = robot_type;
  }

  omni_plan::pddl::Plan
  merge(const std::vector<omni_plan::pddl::Plan> &plans) const {
    return this->merge_plans(plans);
  }
};

class MrtaPlannerTest : public ::testing::Test {
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<TestMrtaPlanner> planner_;
  std::shared_ptr<FakeSubPlanner> sub_planner_;
  std::shared_ptr<FakeTaskAllocator> allocator_;
  std::shared_ptr<TestAction> action_;

  void SetUp() override {
    static int counter = 0;
    node_ = std::make_shared<rclcpp::Node>(
        "test_mrta_planner_" + std::to_string(counter++));
    planner_ = std::make_unique<TestMrtaPlanner>();
    action_ = std::make_shared<TestAction>(
        "a", std::vector<std::pair<std::string, std::string>>{});
    sub_planner_ = std::make_shared<FakeSubPlanner>(action_);
    allocator_ = std::make_shared<FakeTaskAllocator>();
    planner_->inject(node_, sub_planner_, allocator_, "robot");
  }

  static Problem make_three_robot_problem() {
    std::vector<std::pair<std::string, std::string>> robots = {
        {"r0", "robot"}, {"r1", "robot"}, {"r2", "robot"}};
    std::vector<std::pair<std::string, std::string>> objects = {
        {"o0", "obj"}, {"o1", "obj"}, {"o2", "obj"}};
    std::vector<Predicate> goals = {Predicate("done", {"o0"}),
                                    Predicate("done", {"o1"}),
                                    Predicate("done", {"o2"})};
    return make_problem(robots, objects, {}, goals);
  }
};

TEST_F(MrtaPlannerTest, DecomposesGoalsPerTeam) {
  auto p = make_three_robot_problem();
  allocator_->allocation = {{{"r0", "r1"}, {0, 1}}, {{"r2"}, {2}}};

  auto plan = planner_->generate_plan(omni_plan::pddl::Domain{}, p);

  EXPECT_TRUE(plan.has_solution());

  auto records = sub_planner_->records();
  ASSERT_EQ(records.size(), 2u);
  bool found_shared = false;
  bool found_solo = false;
  for (const auto &record : records) {
    if (has_exact_goals(record, {"done(o0)", "done(o1)"})) {
      found_shared = true;
      EXPECT_EQ(record.robots, (std::vector<std::string>{"r0", "r1"}));
    }
    if (has_exact_goals(record, {"done(o2)"})) {
      found_solo = true;
      EXPECT_EQ(record.robots, (std::vector<std::string>{"r2"}));
    }
  }
  EXPECT_TRUE(found_shared);
  EXPECT_TRUE(found_solo);
}

TEST_F(MrtaPlannerTest, MergePreservesInsertionOrderForEqualStartTimes) {
  std::vector<omni_plan::pddl::Plan> plans;
  for (int p = 0; p < 2; ++p) {
    omni_plan::pddl::Plan plan;
    plan.set_has_solution(true);
    for (int i = 0; i < 10; ++i) {
      const int id = p * 10 + i;
      auto action = std::make_shared<TestAction>(
          "a" + std::to_string(id),
          std::vector<std::pair<std::string, std::string>>{});
      plan.add_action(action, {std::to_string(id)}, 0.0f);
    }
    plans.push_back(plan);
  }

  auto merged = planner_->merge(plans);

  ASSERT_EQ(merged.size(), 20u);
  for (size_t i = 0; i < merged.size(); ++i) {
    EXPECT_EQ(merged.get_action_params(i)[0], std::to_string(i));
  }
}

TEST_F(MrtaPlannerTest, FailingSubTeamYieldsNoSolution) {
  auto p = make_three_robot_problem();
  allocator_->allocation = {{{"r0"}, {0}}, {{"r1"}, {1}}, {{"r2"}, {2}}};
  sub_planner_->fail_pred_ = [](const omni_plan::pddl::Problem &problem) {
    for (const auto &goal : problem.get_goals()) {
      if (pred_key(goal) == "done(o1)") {
        return true;
      }
    }
    return false;
  };

  auto plan = planner_->generate_plan(omni_plan::pddl::Domain{}, p);

  EXPECT_FALSE(plan.has_solution());
}

TEST_F(MrtaPlannerTest, GoalReferencingRobotOutsideTeamIsUncovered) {
  std::vector<std::pair<std::string, std::string>> robots = {
      {"r0", "robot"}, {"r1", "robot"}, {"r2", "robot"}};
  std::vector<std::pair<std::string, std::string>> objects = {
      {"o0", "obj"}, {"o1", "obj"}};
  std::vector<Predicate> goals = {Predicate("done", {"o0"}),
                                  Predicate("holding", {"r0", "o1"})};
  auto p = make_problem(robots, objects, {}, goals);

  // Problem goals are stored in a set, so locate the indices explicitly.
  std::set<Predicate> sorted_goals(p.get_goals().begin(), p.get_goals().end());
  int done_idx = -1;
  int holding_idx = -1;
  int i = 0;
  for (const auto &goal : sorted_goals) {
    if (goal.get_name() == "done") {
      done_idx = i;
    } else if (goal.get_name() == "holding") {
      holding_idx = i;
    }
    ++i;
  }
  ASSERT_GE(done_idx, 0);
  ASSERT_GE(holding_idx, 0);

  // done(o0) succeeds on team {r1}; holding(r0,o1) is given to team {r2},
  // which does not contain r0, so it can never be covered.
  allocator_->allocation = {{{"r1"}, {done_idx}}, {{"r2"}, {holding_idx}}};

  auto plan = planner_->generate_plan(omni_plan::pddl::Domain{}, p);

  EXPECT_FALSE(plan.has_solution());
}

TEST_F(MrtaPlannerTest, DuplicateGoalAcrossTeamsIsCountedOnce) {
  auto p = make_three_robot_problem();
  // Goal 1 is offered to two teams; it must be planned once (by team r0).
  allocator_->allocation = {{{"r0"}, {0, 1}}, {{"r1"}, {1, 2}}};

  auto plan = planner_->generate_plan(omni_plan::pddl::Domain{}, p);

  EXPECT_TRUE(plan.has_solution());
  EXPECT_EQ(plan.size(), 3u);
}

// =============================================================================
// main
// =============================================================================
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
