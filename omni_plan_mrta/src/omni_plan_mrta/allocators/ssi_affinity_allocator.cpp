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

#include <limits>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "omni_plan_mrta/allocators/ssi_affinity_allocator.hpp"

namespace omni_plan_mrta {

// ---------------------------------------------------------------------------
// Sequential Single-Item Auction (SSI) with 1-hop Affinity Bidding
// SSI framework — Gerkey, B. P. & Matarić, M. J. (2004). A formal analysis
// and taxonomy of task allocation in multi-robot systems. The International
// Journal of Robotics Research, 23(9), 939–954.
// https://doi.org/10.1177/0278364904045564
// 1-hop affinity bidding heuristic is a custom extension.
// ---------------------------------------------------------------------------
// Bid for (robot_i, goal_j):
//   affinity(i, j) = |{f ∈ S₀ | robot_i ∈ args(f) ∧ args(g_j) ∩ args(f) ≠ ∅}|
//   score(i, j)    = affinity(i, j) − load(i)
//
// Limitation: only direct co-mention counts. Prefer GreedyAuctionAllocator
// when indirect proximity (robot → location → object → goal) matters.
// ---------------------------------------------------------------------------

/// Computes the 1-hop co-mention affinity of robot_i with goal_j.
/// Counts initial-state facts that contain both robot_i and at least one
/// non-robot argument of goal_j.
static int compute_affinity(const std::string &robot_name,
                            const omni_plan::pddl::Predicate &goal,
                            const std::set<std::string> &all_robot_names,
                            const std::set<omni_plan::pddl::Predicate> &facts) {

  // Non-robot arguments of the goal
  std::set<std::string> goal_objects;
  for (const auto &arg : goal.get_args()) {
    if (!all_robot_names.count(arg)) {
      goal_objects.insert(arg);
    }
  }
  if (goal_objects.empty()) {
    return 0;
  }

  int count = 0;
  for (const auto &fact : facts) {
    const auto &args = fact.get_args();
    bool has_robot = false;
    bool has_goal_obj = false;
    for (const auto &arg : args) {
      if (arg == robot_name) {
        has_robot = true;
      }
      if (goal_objects.count(arg)) {
        has_goal_obj = true;
      }
    }
    if (has_robot && has_goal_obj) {
      ++count;
    }
  }
  return count;
}

SsiAffinityAllocator::SsiAffinityAllocator() : TaskAllocator() {}

std::unordered_map<std::string, RobotAllocation> SsiAffinityAllocator::allocate(
    const std::vector<std::string> &robots,
    const std::vector<omni_plan::pddl::Predicate> &goals,
    const omni_plan::pddl::Problem &problem,
    const std::unordered_map<std::string,
                             std::shared_ptr<omni_plan::pddl::Action>>
        & /*actions*/) const {
  const int N = static_cast<int>(robots.size());
  const int M = static_cast<int>(goals.size());

  std::unordered_map<std::string, RobotAllocation> result;
  for (const auto &r : robots) {
    result[r] = RobotAllocation{};
  }

  if (N == 0 || M == 0) {
    return result;
  }

  const std::set<std::string> all_robot_names(robots.begin(), robots.end());
  const auto &facts = problem.get_facts();

  // Affinity matrix A[i][j]
  std::vector<std::vector<int>> A(N, std::vector<int>(M, 0));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      A[i][j] = compute_affinity(robots[i], goals[j], all_robot_names, facts);
    }
  }

  // SSI greedy auction: score(i, j) = A[i][j] - load[i]
  std::vector<int> load(N, 0);
  std::vector<bool> unassigned(M, true);

  for (int step = 0; step < M; ++step) {
    int best_robot = -1;
    int best_goal = -1;
    int best_score = std::numeric_limits<int>::min();

    for (int j = 0; j < M; ++j) {
      if (!unassigned[j]) {
        continue;
      }
      for (int i = 0; i < N; ++i) {
        const int score = A[i][j] - load[i];
        if (score > best_score) {
          best_score = score;
          best_robot = i;
          best_goal = j;
        }
      }
    }

    if (best_goal >= 0 && best_robot >= 0) {
      result[robots[static_cast<size_t>(best_robot)]].goal_indices.push_back(
          best_goal);
      unassigned[best_goal] = false;
      ++load[best_robot];
    }
  }

  return result;
}

} // namespace omni_plan_mrta

PLUGINLIB_EXPORT_CLASS(omni_plan_mrta::SsiAffinityAllocator,
                       omni_plan_mrta::TaskAllocator)
