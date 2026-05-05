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
#include <limits>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "omni_plan_mrta/allocation_utils.hpp"
#include "omni_plan_mrta/allocators/cbba_allocator.hpp"

namespace omni_plan_mrta {

// ============================================================================
// CBBA — Consensus-Based Bundle Algorithm
// Reference: Choi et al. (2009). IEEE T-RO 25(4), 912–926.
// ============================================================================

namespace {

/// Canonical string key for a ground fact: "name(a1,a2,...)".
std::string fact_key(const std::string &pred_name,
                     const std::vector<std::string> &args) {
  std::string k = pred_name + "(";
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0) {
      k += ',';
    }
    k += args[i];
  }
  return k + ')';
}

/// A delete-relaxed ground action reduced to positive keys.
struct GroundAction {
  std::vector<std::string> pre;
  std::vector<std::string> eff;
};

/// Applies a symbolic → object substitution.
std::vector<std::string>
apply_subst(const std::vector<std::string> &args,
            const std::unordered_map<std::string, std::string> &subst) {
  std::vector<std::string> out;
  out.reserve(args.size());
  for (const auto &a : args) {
    auto it = subst.find(a);
    out.push_back(it != subst.end() ? it->second : a);
  }
  return out;
}

/// Recursively enumerates all complete parameter substitutions.
void enumerate_substs(
    const std::vector<std::pair<std::string, std::string>> &params,
    const std::unordered_map<std::string, std::vector<std::string>> &by_type,
    std::size_t idx, std::unordered_map<std::string, std::string> &cur,
    std::vector<std::unordered_map<std::string, std::string>> &out,
    std::size_t max_out) {
  if (out.size() >= max_out) {
    return;
  }
  if (idx == params.size()) {
    out.push_back(cur);
    return;
  }
  const auto &pname = params[idx].first;
  const auto &ptype = params[idx].second;
  auto it = by_type.find(ptype);
  if (it == by_type.end()) {
    return;
  }
  for (const auto &obj : it->second) {
    if (out.size() >= max_out) {
      break;
    }
    cur[pname] = obj;
    enumerate_substs(params, by_type, idx + 1, cur, out, max_out);
  }
  cur.erase(pname);
}

/// Grounds all action templates and returns those relevant to robot_name.
std::vector<GroundAction> ground_actions_for_robot(
    const std::string &robot_name,
    const std::set<omni_plan::pddl::Object> &problem_objects,
    const std::unordered_map<
        std::string, std::shared_ptr<omni_plan::pddl::Action>> &templates,
    std::size_t max_per_action = 2048) {
  std::unordered_map<std::string, std::vector<std::string>> by_type;
  for (const auto &obj : problem_objects) {
    by_type[obj.get_type()].push_back(obj.get_name());
  }

  std::vector<GroundAction> result;

  for (const auto &[action_name, tmpl] : templates) {
    const auto raw_params = tmpl->get_parameters();

    if (raw_params.empty()) {
      GroundAction ga;
      const std::unordered_map<std::string, std::string> empty_subst;
      for (const auto &cond : tmpl->get_conditions()) {
        if (!cond.is_negated()) {
          ga.pre.push_back(fact_key(cond.get_name(),
                                    apply_subst(cond.get_args(), empty_subst)));
        }
      }
      for (const auto &eff : tmpl->get_effects()) {
        if (!eff.is_negated()) {
          ga.eff.push_back(fact_key(eff.get_name(),
                                    apply_subst(eff.get_args(), empty_subst)));
        }
      }
      result.push_back(std::move(ga));
      continue;
    }

    std::vector<std::pair<std::string, std::string>> params;
    params.reserve(raw_params.size());
    for (const auto &p : raw_params) {
      params.emplace_back(p.get_name(), p.get_type());
    }

    std::vector<std::unordered_map<std::string, std::string>> substs;
    std::unordered_map<std::string, std::string> cur;
    enumerate_substs(params, by_type, 0, cur, substs, max_per_action);

    for (const auto &subst : substs) {
      bool involves_robot = false;
      for (const auto &[pname, pval] : subst) {
        if (pval == robot_name) {
          involves_robot = true;
          break;
        }
      }
      if (!involves_robot) {
        continue;
      }

      GroundAction ga;
      for (const auto &cond : tmpl->get_conditions()) {
        if (!cond.is_negated()) {
          ga.pre.push_back(
              fact_key(cond.get_name(), apply_subst(cond.get_args(), subst)));
        }
      }
      for (const auto &eff : tmpl->get_effects()) {
        if (!eff.is_negated()) {
          ga.eff.push_back(
              fact_key(eff.get_name(), apply_subst(eff.get_args(), subst)));
        }
      }
      result.push_back(std::move(ga));
    }
  }

  return result;
}

/// Delete-relaxed Bellman-Ford propagation (h_add or h_max).
std::unordered_map<std::string, int>
compute_relaxed_costs(const std::set<omni_plan::pddl::Predicate> &init_facts,
                      const std::vector<GroundAction> &ground_acts,
                      bool use_h_max) {
  const int kInf = std::numeric_limits<int>::max() / 2;

  std::unordered_map<std::string, int> cost;
  cost.reserve(init_facts.size() * 4);
  for (const auto &f : init_facts) {
    cost[fact_key(f.get_name(), f.get_args())] = 0;
  }

  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto &ga : ground_acts) {
      int pre_agg = 0;
      bool feasible = true;
      for (const auto &pre_key : ga.pre) {
        auto it = cost.find(pre_key);
        if (it == cost.end() || it->second >= kInf) {
          feasible = false;
          break;
        }
        if (use_h_max) {
          pre_agg = std::max(pre_agg, it->second);
        } else {
          if (pre_agg > kInf - it->second) {
            feasible = false;
            break;
          }
          pre_agg += it->second;
        }
      }
      if (!feasible) {
        continue;
      }

      const int action_cost = 1 + pre_agg;
      for (const auto &eff_key : ga.eff) {
        auto it = cost.find(eff_key);
        if (it == cost.end() || action_cost < it->second) {
          cost[eff_key] = action_cost;
          changed = true;
        }
      }
    }
  }

  return cost;
}

} // namespace

// ============================================================================
// CbbaAllocator implementation
// ============================================================================

CbbaAllocator::CbbaAllocator(bool use_h_max)
    : TaskAllocator(), use_h_max_(use_h_max) {
  this->add_ros_parameters({
      {"use_h_max", false, this->use_h_max_},
  });
}

std::vector<TeamAllocation> CbbaAllocator::allocate(
    const std::vector<std::string> &robots,
    const std::vector<omni_plan::pddl::Predicate> &goals,
    const omni_plan::pddl::Problem &problem,
    const std::unordered_map<
        std::string, std::shared_ptr<omni_plan::pddl::Action>> &actions) const {

  const int N = static_cast<int>(robots.size());
  const int M = static_cast<int>(goals.size());

  std::vector<TeamAllocation> result;
  if (N == 0 || M == 0) {
    return result;
  }

  const int kInf = std::numeric_limits<int>::max() / 2;
  const std::set<std::string> all_robot_names(robots.begin(), robots.end());

  // Step 1: Per-robot delete-relaxed cost maps
  std::vector<std::unordered_map<std::string, int>> cost_maps(N);
  for (int i = 0; i < N; ++i) {
    std::set<omni_plan::pddl::Predicate> robot_facts;
    for (const auto &f : problem.get_facts()) {
      bool mentions_other = false;
      for (const auto &arg : f.get_args()) {
        if (all_robot_names.count(arg) && arg != robots[i]) {
          mentions_other = true;
          break;
        }
      }
      if (!mentions_other) {
        robot_facts.insert(f);
      }
    }
    const auto ground_acts =
        ground_actions_for_robot(robots[i], problem.get_objects(), actions);
    cost_maps[i] = compute_relaxed_costs(robot_facts, ground_acts, use_h_max_);
  }

  // Step 2: BFS-distance proximity (secondary cost term)
  //
  // Before running BFS, collect all non-robot args that directly co-occur with
  // a robot name in the init facts (robot_pos_args). These are "occupied"
  // locations — e.g. kitchen_counter in (robot_at waiter1 kitchen_counter).
  // They are excluded from BFS targets so that a robot trivially at one arg of
  // a multi-arg goal (e.g. waiter1 already at kitchen_counter for goal
  // (order_ready table3 kitchen_counter)) does not gain a 1-hop advantage over
  // a robot that is genuinely closer to the goal's unoccupied primary locations
  // (e.g. waiter2 near table3 in dining_room).
  const auto adj = build_object_adjacency(problem.get_facts());
  const auto robot_pos_args =
      build_robot_pos_args(problem.get_facts(), all_robot_names);

  const int bfs_unreachable = std::numeric_limits<int>::max() / 2;
  int max_finite_bfs = 0;

  // Single-pass: compute BFS distances and track the max finite value.
  // bfs_capped is populated with the raw distance; unreachable cells are
  // fixed up to dist_scale in a second pass once dist_scale is known.
  std::vector<std::vector<int>> bfs_capped(
      N, std::vector<int>(M, bfs_unreachable));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      const int d =
          compute_bfs_distance(robots[i], goals[j], adj, robot_pos_args);
      bfs_capped[i][j] = d;
      if (d < bfs_unreachable && d > max_finite_bfs) {
        max_finite_bfs = d;
      }
    }
  }

  // dist_scale > max_finite_bfs ensures h-cost always dominates BFS distance.
  const int dist_scale = max_finite_bfs + 1;

  // Cap unreachable entries now that dist_scale is known.
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      if (bfs_capped[i][j] >= bfs_unreachable) {
        bfs_capped[i][j] = dist_scale;
      }
    }
  }

  // Step 3: Bid matrix
  //
  //   c[i][j] = -(h_cost(i,j) * dist_scale + bfs_capped(i,j))
  //
  // Higher bid = better robot for goal j. h_cost dominates; BFS breaks ties.
  std::vector<std::string> goal_keys(M);
  for (int j = 0; j < M; ++j) {
    goal_keys[j] = fact_key(goals[j].get_name(), goals[j].get_args());
  }

  int max_abs_cost = 0;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      auto it = cost_maps[i].find(goal_keys[j]);
      if (it != cost_maps[i].end() && it->second < kInf) {
        if (it->second > max_abs_cost) {
          max_abs_cost = it->second;
        }
      }
    }
  }

  // Maximum bid magnitude: -(0 * dist_scale + 0) = 0 (best case).
  // Minimum feasible bid: -(max_abs_cost * dist_scale + dist_scale).
  const int max_bid_magnitude = max_abs_cost * dist_scale + dist_scale;
  // alpha: adding a task to the bundle must never outbid the best singleton.
  const int alpha = max_bid_magnitude / (M + 1) + 1;
  // unreachable_bid: strictly below any feasible bid after the full penalty.
  const int unreachable_bid = -(max_bid_magnitude + alpha * M + 1);

  std::vector<std::vector<int>> c(N, std::vector<int>(M, unreachable_bid));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      auto it = cost_maps[i].find(goal_keys[j]);
      if (it != cost_maps[i].end() && it->second < kInf) {
        c[i][j] = -(it->second * dist_scale + bfs_capped[i][j]);
      }
    }
  }

  // Step 4: CBBA main loop
  // neg_inf must be strictly below every possible bid value. Compute it as
  // INT_MIN/2 to avoid underflow: unreachable_bid − alpha × M − 1 can
  // overflow when alpha or M are large.
  const int neg_inf = std::numeric_limits<int>::min() / 2;
  std::vector<std::vector<int>> y(N, std::vector<int>(M, neg_inf));
  std::vector<std::vector<int>> z(N, std::vector<int>(M, -1));
  std::vector<std::vector<int>> bundle(N);
  // O(1) bundle-membership lookup: in_bundle[i][j] == true iff goal j is in
  // robot i's bundle.  Avoids the O(bundle_size) std::find in the inner loop.
  std::vector<std::vector<bool>> in_bundle(
      static_cast<size_t>(N), std::vector<bool>(static_cast<size_t>(M), false));

  const int max_iter = 3 * N * M + 1;
  for (int iter = 0; iter < max_iter; ++iter) {
    // Bundle phase
    for (int i = 0; i < N; ++i) {
      while (true) {
        int best_j = -1;
        int best_bid = neg_inf;

        for (int j = 0; j < M; ++j) {
          if (in_bundle[static_cast<size_t>(i)][static_cast<size_t>(j)]) {
            continue;
          }
          const int bid = c[i][j] - alpha * static_cast<int>(bundle[i].size());
          if (bid > y[i][j] && bid > best_bid) {
            best_bid = bid;
            best_j = j;
          }
        }

        if (best_j < 0) {
          break;
        }

        bundle[i].push_back(best_j);
        in_bundle[static_cast<size_t>(i)][static_cast<size_t>(best_j)] = true;
        y[i][best_j] = best_bid;
        z[i][best_j] = i;
      }
    }

    // Consensus phase
    bool any_change = false;
    for (int j = 0; j < M; ++j) {
      int global_bid = neg_inf;
      int global_winner = -1;
      // Use the current agreed-upon winner as a tiebreaker: if two robots
      // have equal bids, the one already holding the task keeps it. Without
      // this, equal-cost goals always revert to robot 0 (lowest index) after
      // the bid table is synchronised, preventing other robots from ever
      // retaining a task they legitimately won.
      const int stable_winner = z[0][j];
      for (int i = 0; i < N; ++i) {
        if (y[i][j] > global_bid ||
            (y[i][j] == global_bid && i == stable_winner)) {
          global_bid = y[i][j];
          global_winner = i;
        }
      }
      if (global_winner < 0) {
        continue;
      }

      for (int i = 0; i < N; ++i) {
        if (y[i][j] < global_bid || z[i][j] != global_winner) {
          if (z[i][j] == i) {
            auto &b = bundle[i];
            b.erase(std::remove(b.begin(), b.end(), j), b.end());
            in_bundle[static_cast<size_t>(i)][static_cast<size_t>(j)] = false;
            any_change = true;
          }
          y[i][j] = global_bid;
          z[i][j] = global_winner;
        }
      }
    }

    if (!any_change) {
      break;
    }
  }

  // Step 5: Build allocation from consensus winner table
  // robot_idx -> index in result vector (-1 = not yet inserted)
  std::vector<int> robot_result_idx(static_cast<size_t>(N), -1);

  for (int j = 0; j < M; ++j) {
    const int winner = z[0][j];
    if (winner >= 0 && winner < N) {
      const size_t wi = static_cast<size_t>(winner);
      if (robot_result_idx[wi] < 0) {
        robot_result_idx[wi] = static_cast<int>(result.size());
        result.push_back(TeamAllocation{{robots[wi]}, {}});
      }
      result[static_cast<size_t>(robot_result_idx[wi])].goal_indices.push_back(
          j);
    }
  }

  // Step 6: Distribute unassigned goals (load-balancing fallback)
  std::vector<int> load(N, 0);
  for (int i = 0; i < N; ++i) {
    if (robot_result_idx[i] >= 0) {
      load[i] = static_cast<int>(
          result[static_cast<size_t>(robot_result_idx[i])].goal_indices.size());
    }
  }
  for (int j = 0; j < M; ++j) {
    if (z[0][j] >= 0) {
      continue;
    }
    const int min_robot = static_cast<int>(
        std::min_element(load.begin(), load.end()) - load.begin());
    const size_t mi = static_cast<size_t>(min_robot);
    if (robot_result_idx[mi] < 0) {
      robot_result_idx[mi] = static_cast<int>(result.size());
      result.push_back(TeamAllocation{{robots[mi]}, {}});
    }
    result[static_cast<size_t>(robot_result_idx[mi])].goal_indices.push_back(j);
    ++load[min_robot];
  }

  // Step 7: Rebalance load — drive every robot's goal count to within 1 of
  // M/N (the optimal spread for N robots and M goals).
  //
  // Each iteration picks the most-loaded donor and least-loaded receiver.
  // If donor.load > receiver.load + 1 a steal is possible; we pick the goal
  // from the donor whose cost delta for the receiver is smallest (minimum
  // quality loss). Iteration stops when either the spread is already ≤ 1
  // (no donor qualifies) or no h-cost-reachable goal can be moved (accept
  // remaining imbalance rather than force an unreachable assignment).
  if (M >= N) {
    while (true) {
      // Receiver: robot with the minimum current load.
      const int receiver = static_cast<int>(
          std::min_element(load.begin(), load.end()) - load.begin());

      // Donor: most-loaded robot that still has > receiver.load + 1 goals.
      // If no such donor exists the spread is already ≤ 1 — done.
      int donor = -1;
      for (int i = 0; i < N; ++i) {
        if (load[i] > load[receiver] + 1) {
          if (donor < 0 || load[i] > load[donor]) {
            donor = i;
          }
        }
      }
      if (donor < 0) {
        break; // spread ≤ 1: optimally balanced
      }

      // Among the donor's goals find the one the receiver can reach (finite
      // h-cost) with the smallest quality delta.
      const auto &donor_goals =
          result[static_cast<size_t>(robot_result_idx[donor])].goal_indices;

      int best_steal_pos = -1;
      int best_delta = std::numeric_limits<int>::max();
      for (int pos = 0; pos < static_cast<int>(donor_goals.size()); ++pos) {
        const int j = donor_goals[static_cast<size_t>(pos)];
        if (c[receiver][j] <= unreachable_bid) {
          continue; // receiver cannot achieve this goal
        }
        const int delta = c[donor][j] - c[receiver][j];
        if (delta < best_delta) {
          best_delta = delta;
          best_steal_pos = pos;
        }
      }

      if (best_steal_pos < 0) {
        break; // receiver cannot reach any donor goal; accept remaining
               // imbalance
      }

      const int stolen_goal = donor_goals[static_cast<size_t>(best_steal_pos)];

      // Remove from donor.
      auto &dg =
          result[static_cast<size_t>(robot_result_idx[donor])].goal_indices;
      dg.erase(dg.begin() + best_steal_pos);
      --load[donor];

      // Add to receiver (create its TeamAllocation entry on first assignment).
      if (robot_result_idx[receiver] < 0) {
        robot_result_idx[receiver] = static_cast<int>(result.size());
        result.push_back(TeamAllocation{{robots[receiver]}, {}});
      }
      result[static_cast<size_t>(robot_result_idx[receiver])]
          .goal_indices.push_back(stolen_goal);
      ++load[receiver];
    }
  }

  return result;
}

} // namespace omni_plan_mrta

PLUGINLIB_EXPORT_CLASS(omni_plan_mrta::CbbaAllocator,
                       omni_plan_mrta::TaskAllocator)
