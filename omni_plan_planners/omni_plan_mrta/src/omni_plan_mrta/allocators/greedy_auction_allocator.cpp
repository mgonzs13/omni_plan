// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "omni_plan_mrta/allocation_utils.hpp"
#include "omni_plan_mrta/allocators/greedy_auction_allocator.hpp"

namespace omni_plan_mrta {

// ---------------------------------------------------------------------------
// SSI with BFS-Distance Bidding
// SSI framework — Gerkey, B. P. & Matarić, M. J. (2004). A formal analysis
// and taxonomy of task allocation in multi-robot systems. The International
// Journal of Robotics Research, 23(9), 939–954.
// https://doi.org/10.1177/0278364904045564
// BFS-distance bidding heuristic is a custom extension.
// ---------------------------------------------------------------------------

GreedyAuctionAllocator::GreedyAuctionAllocator() : TaskAllocator() {}

std::vector<TeamAllocation> GreedyAuctionAllocator::allocate(
    const std::vector<std::string> &robots,
    const std::vector<omni_plan::pddl::Predicate> &goals,
    const omni_plan::pddl::Problem &problem,
    const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        & /*actions*/) const {
  const int N = static_cast<int>(robots.size());
  const int M = static_cast<int>(goals.size());

  std::vector<TeamAllocation> result;
  if (N == 0 || M == 0) {
    return result;
  }

  const auto adj = build_object_adjacency(problem.get_facts());

  // Exclude robot-occupied args from BFS targets (same rationale as CBBA).
  const std::set<std::string> all_robot_names(robots.begin(), robots.end());
  const auto robot_pos_args =
      build_robot_pos_args(problem.get_facts(), all_robot_names);

  // Both the sentinel and the reachability check use the shared BFS
  // unreachable value (INT_MAX/2). A smaller sentinel such as N*M+1 could be
  // outscored by a finite distance larger than it, making an unreachable pair
  // look better than a distant reachable one.
  const int unreachable = std::numeric_limits<int>::max() / 2;
  std::vector<std::vector<int>> D(N, std::vector<int>(M, unreachable));
  int max_finite_dist = 0;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      const int d =
          compute_bfs_distance(robots[i], goals[j], adj, robot_pos_args);
      if (d < unreachable) {
        D[i][j] = d;
        if (d > max_finite_dist) {
          max_finite_dist = d;
        }
      }
    }
  }

  const int load_coeff = max_finite_dist + 1;

  std::vector<int> load(N, 0);
  std::vector<bool> unassigned(M, true);
  // robot_idx -> index in result vector (-1 = not yet inserted)
  std::vector<int> robot_result_idx(static_cast<size_t>(N), -1);

  for (int step = 0; step < M; ++step) {
    int best_robot = -1;
    int best_goal = -1;
    int best_score = std::numeric_limits<int>::min();

    for (int j = 0; j < M; ++j) {
      if (!unassigned[j]) {
        continue;
      }
      for (int i = 0; i < N; ++i) {
        const int score = -D[i][j] - load_coeff * load[i];
        if (score > best_score) {
          best_score = score;
          best_robot = i;
          best_goal = j;
        }
      }
    }

    if (best_goal >= 0 && best_robot >= 0) {
      const size_t ri = static_cast<size_t>(best_robot);
      if (robot_result_idx[ri] < 0) {
        robot_result_idx[ri] = static_cast<int>(result.size());
        result.push_back(TeamAllocation{{robots[ri]}, {}});
      }
      result[static_cast<size_t>(robot_result_idx[ri])].goal_indices.push_back(
          best_goal);
      unassigned[best_goal] = false;
      ++load[best_robot];
    }
  }

  return result;
}

} // namespace omni_plan_mrta

PLUGINLIB_EXPORT_CLASS(omni_plan_mrta::GreedyAuctionAllocator,
                       omni_plan_mrta::TaskAllocator)
