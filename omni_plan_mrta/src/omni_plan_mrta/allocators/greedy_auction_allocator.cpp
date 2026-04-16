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
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

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

/// Builds an object co-occurrence adjacency list from initial-state facts.
static std::unordered_map<std::string, std::vector<std::string>>
build_object_adjacency(const std::set<omni_plan::pddl::Predicate> &facts) {
  std::unordered_map<std::string, std::vector<std::string>> adj;
  for (const auto &fact : facts) {
    const auto &args = fact.get_args();
    for (size_t a = 0; a < args.size(); ++a) {
      for (size_t b = a + 1; b < args.size(); ++b) {
        adj[args[a]].push_back(args[b]);
        adj[args[b]].push_back(args[a]);
      }
    }
  }
  return adj;
}

/// Returns the BFS distance from robot_name to the nearest non-robot argument
/// of goal in the object co-occurrence graph. Returns INT_MAX/2 if unreachable.
static int compute_bfs_distance(
    const std::string &robot_name, const omni_plan::pddl::Predicate &goal,
    const std::unordered_map<std::string, std::vector<std::string>> &adj) {
  const auto &goal_args = goal.get_args();
  std::set<std::string> targets(goal_args.begin(), goal_args.end());
  targets.erase(robot_name);

  if (targets.empty()) {
    return 0;
  }

  std::unordered_map<std::string, int> visited;
  std::queue<std::string> frontier;
  visited[robot_name] = 0;
  frontier.push(robot_name);

  while (!frontier.empty()) {
    const std::string curr = std::move(frontier.front());
    frontier.pop();
    const int d = visited.at(curr);

    if (targets.count(curr)) {
      return d;
    }

    auto it = adj.find(curr);
    if (it == adj.end()) {
      continue;
    }
    for (const auto &nb : it->second) {
      if (!visited.count(nb)) {
        visited[nb] = d + 1;
        if (targets.count(nb)) {
          return d + 1;
        }
        frontier.push(nb);
      }
    }
  }

  return std::numeric_limits<int>::max() / 2;
}

GreedyAuctionAllocator::GreedyAuctionAllocator() : TaskAllocator() {}

std::unordered_map<std::string, RobotAllocation>
GreedyAuctionAllocator::allocate(
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

  const auto adj = build_object_adjacency(problem.get_facts());

  const int unreachable = N * M + 1;
  std::vector<std::vector<int>> D(N, std::vector<int>(M, unreachable));
  int max_finite_dist = 0;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      const int d = compute_bfs_distance(robots[i], goals[j], adj);
      if (d < std::numeric_limits<int>::max() / 2) {
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
      result[robots[static_cast<size_t>(best_robot)]].goal_indices.push_back(
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
