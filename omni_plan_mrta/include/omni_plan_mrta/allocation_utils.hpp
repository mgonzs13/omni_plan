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

#ifndef OMNI_PLAN_MRTA__ALLOCATION_UTILS_HPP_
#define OMNI_PLAN_MRTA__ALLOCATION_UTILS_HPP_

#include <algorithm>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/predicate.hpp"

namespace omni_plan_mrta {

/**
 * @brief Builds a co-occurrence adjacency list from initial-state facts.
 *
 * @details Two argument names are adjacent if they appear together in any
 * single fact. This graph encodes spatial and relational structure without
 * requiring knowledge of predicate names: locations adjacent to rooms, rooms
 * adjacent to connected locations, robots adjacent to their current location,
 * etc.
 *
 * @param facts The set of initial-state predicates.
 * @return Map from argument name to its list of co-occurring argument names.
 */
inline std::unordered_map<std::string, std::vector<std::string>>
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
  // Deduplicate each neighbor list so BFS does not re-visit the same node
  // multiple times when the same arg pair co-appears in several facts.
  for (auto &[node, neighbors] : adj) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()),
                    neighbors.end());
  }
  return adj;
}

/**
 * @brief Collects non-robot argument values that directly co-occur with a
 * robot name in any initial-state fact.
 *
 * @details These are the "robot-position args": objects/locations that appear
 * alongside a robot in a fact such as @c robot_at(waiter1, kitchen_counter).
 * They represent slots in the state that are already attended by some robot.
 * Passing this set to @c compute_bfs_distance excludes occupied locations from
 * the BFS target set, preventing a robot that is trivially co-located with one
 * argument of a multi-arg goal from gaining an unfair BFS advantage over a
 * robot that is genuinely closer to the goal's primary (unoccupied) locations.
 *
 * @param facts        The set of initial-state predicates.
 * @param robot_names  The set of robot name strings.
 * @return Set of non-robot argument values co-occurring with a robot.
 */
inline std::set<std::string>
build_robot_pos_args(const std::set<omni_plan::pddl::Predicate> &facts,
                     const std::set<std::string> &robot_names) {
  std::set<std::string> pos_args;
  for (const auto &f : facts) {
    const auto &fargs = f.get_args();
    bool has_robot = false;
    for (const auto &a : fargs) {
      if (robot_names.count(a)) {
        has_robot = true;
        break;
      }
    }
    if (!has_robot) {
      continue;
    }
    for (const auto &a : fargs) {
      if (!robot_names.count(a)) {
        pos_args.insert(a);
      }
    }
  }
  return pos_args;
}

/**
 * @brief BFS distance sum from @p robot_name to the unoccupied arguments of
 * @p goal in the object co-occurrence graph.
 *
 * @details Runs a single BFS from the robot and records the distance to each
 * distinct target. The target set is computed as follows:
 *
 * 1. Start with all goal arguments excluding @p robot_name itself.
 * 2. Remove any argument that appears in @p robot_pos_args (i.e. an argument
 *    that is already the current location/state of some robot). These args are
 *    "occupied" — the BFS advantage a robot gains from already being there is
 *    trivial and does not reflect genuine suitability for the whole goal.
 * 3. If all goal args are occupied the full set (minus the robot name) is used
 *    as a fallback, so every goal can always be scored.
 *
 * The returned value is the **sum** of per-target BFS distances, each
 * individually capped at INT_MAX/2 when unreachable.
 *
 * @param robot_name     The robot whose position is the BFS source.
 * @param goal           Goal predicate whose args are targets.
 * @param adj            Co-occurrence adjacency list from
 * build_object_adjacency.
 * @param robot_pos_args Args currently occupied by some robot (from
 *                       build_robot_pos_args). Excluded from primary targets.
 * @return Sum of per-target BFS hop counts (each capped at INT_MAX/2).
 */
inline int compute_bfs_distance(
    const std::string &robot_name, const omni_plan::pddl::Predicate &goal,
    const std::unordered_map<std::string, std::vector<std::string>> &adj,
    const std::set<std::string> &robot_pos_args = {}) {
  const int kUnreachable = std::numeric_limits<int>::max() / 2;

  const auto &goal_args = goal.get_args();

  // Primary targets: goal args that are neither the querying robot nor a
  // robot-occupied location. A robot sitting at kitchen_counter should not
  // gain a 1-hop advantage for a goal that also involves table3 — the
  // occupied arg is trivially near and contributes nothing discriminating.
  std::set<std::string> targets;
  for (const auto &a : goal_args) {
    if (a != robot_name && !robot_pos_args.count(a)) {
      targets.insert(a);
    }
  }

  // Fallback: if every goal arg is occupied, use the full set minus robot_name
  // so the goal can still be scored.
  if (targets.empty()) {
    for (const auto &a : goal_args) {
      if (a != robot_name) {
        targets.insert(a);
      }
    }
  }

  if (targets.empty()) {
    return 0;
  }

  // Single BFS from robot_name; record distance to every target found.
  std::unordered_map<std::string, int> visited;
  std::queue<std::string> frontier;
  visited[robot_name] = 0;
  frontier.push(robot_name);

  std::unordered_map<std::string, int> target_dist;

  while (!frontier.empty() && target_dist.size() < targets.size()) {
    const std::string curr = std::move(frontier.front());
    frontier.pop();
    const int d = visited.at(curr);

    if (targets.count(curr)) {
      target_dist[curr] = d;
    }

    auto it = adj.find(curr);
    if (it == adj.end()) {
      continue;
    }
    for (const auto &nb : it->second) {
      if (!visited.count(nb)) {
        visited[nb] = d + 1;
        frontier.push(nb);
      }
    }
  }

  // Sum distances to all targets; use kUnreachable for any not found.
  // Clamp the running total to avoid overflow.
  int total = 0;
  for (const auto &t : targets) {
    auto it = target_dist.find(t);
    const int d = (it != target_dist.end()) ? it->second : kUnreachable;
    if (total > kUnreachable - d) {
      return kUnreachable; // overflow guard
    }
    total += d;
  }
  return total;
}

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATION_UTILS_HPP_
