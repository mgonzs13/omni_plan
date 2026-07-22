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
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "omni_plan_mrta/allocation_utils.hpp"
#include "omni_plan_mrta/allocators/coalition_formation_allocator.hpp"

namespace omni_plan_mrta {

// ---------------------------------------------------------------------------
// Coalition Formation Allocator
// Phase 1: classify goals as single-robot (SR) or multi-robot (MR) using
//          delete-relaxed reachability analysis per robot.
// Phase 2: reachability-first coalition formation for MR goals.
// Phase 3: load-balanced BFS-distance greedy auction for SR goals.
//
// References:
//   Shehory & Kraus (1998). Methods for task allocation via agent coalition
//   formation. AI, 101(1–2), 165–200.
//   Gerkey & Matarić (2004). A Formal Analysis and Taxonomy of Task Allocation
//   in Multi-Robot Systems. IJRR 23(9), 939–954.
// ---------------------------------------------------------------------------

namespace {

// ---------------------------------------------------------------------------
// Grounding utilities (adapted from CbbaAllocator)
// ---------------------------------------------------------------------------

/// Canonical string key for a ground fact: "name(a1,a2,...)".
static std::string fact_key(const std::string &pred_name,
                            const std::vector<std::string> &args) {
  std::string k = pred_name + '(';
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0) {
      k += ',';
    }
    k += args[i];
  }
  k += ')';
  return k;
}

/// A ground action with its positive preconditions and effects as fact keys.
struct GroundAction {
  std::vector<std::string> pre;
  std::vector<std::string> eff;
};

/// Apply a parameter→object substitution to an argument list.
static std::vector<std::string>
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

/// Recursively enumerate all type-consistent substitutions for @p params.
static void enumerate_substs(
    const std::vector<std::pair<std::string, std::string>> &params,
    const std::unordered_map<std::string, std::vector<std::string>> &by_type,
    size_t idx, std::unordered_map<std::string, std::string> &cur,
    std::vector<std::unordered_map<std::string, std::string>> &out,
    std::size_t max_out) {
  if (out.size() >= max_out) {
    return;
  }
  if (idx == params.size()) {
    out.push_back(cur);
    return;
  }
  const auto &[pname, ptype] = params[idx];
  auto it = by_type.find(ptype);
  if (it == by_type.end()) {
    return;
  }
  for (const auto &obj : it->second) {
    cur[pname] = obj;
    enumerate_substs(params, by_type, idx + 1, cur, out, max_out);
    if (out.size() >= max_out) {
      break; // break (not return) so cur[pname] is erased before unwinding
    }
  }
  cur.erase(pname);
}

/// Ground all action templates that involve @p robot_name as a parameter.
static std::vector<GroundAction> ground_actions_for_robot(
    const std::string &robot_name,
    const std::set<omni_plan::pddl::Object> &problem_objects,
    const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        &templates,
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

/// Delete-relaxed Bellman-Ford propagation (h_add).
/// Returns a cost map; reachable facts have cost < kInf.
static std::unordered_map<std::string, int>
compute_relaxed_costs(const std::set<omni_plan::pddl::Predicate> &init_facts,
                      const std::vector<GroundAction> &ground_acts) {
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
        if (pre_agg > kInf - it->second) {
          feasible = false;
          break;
        }
        pre_agg += it->second;
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

/// Return the initial facts filtered to only include predicates that do NOT
/// mention any robot outside @p coalition_names.
static std::set<omni_plan::pddl::Predicate> filter_facts_for_coalition(
    const std::set<omni_plan::pddl::Predicate> &all_facts,
    const std::set<std::string> &all_robot_names,
    const std::set<std::string> &coalition_names) {
  std::set<omni_plan::pddl::Predicate> filtered;
  for (const auto &f : all_facts) {
    bool mentions_outside = false;
    for (const auto &arg : f.get_args()) {
      if (all_robot_names.count(arg) && !coalition_names.count(arg)) {
        mentions_outside = true;
        break;
      }
    }
    if (!mentions_outside) {
      filtered.insert(f);
    }
  }
  return filtered;
}
// NOTE: build_object_adjacency, build_robot_pos_args, and compute_bfs_distance
// are provided by allocation_utils.hpp (omni_plan_mrta namespace).

// ---------------------------------------------------------------------------
// K-subset enumeration (iterative, lexicographic)
// ---------------------------------------------------------------------------
/// Generates all combinations of size @p k from indices [0, n).
static std::vector<std::vector<int>> combinations(int n, int k) {
  std::vector<std::vector<int>> result;
  if (k <= 0 || k > n) {
    return result;
  }
  std::vector<int> c(static_cast<size_t>(k));
  for (int i = 0; i < k; ++i) {
    c[static_cast<size_t>(i)] = i;
  }
  while (true) {
    result.push_back(c);

    int i = k - 1;
    while (i >= 0 && c[static_cast<size_t>(i)] == n - k + i) {
      --i;
    }
    if (i < 0) {
      break;
    }
    ++c[static_cast<size_t>(i)];
    for (int j = i + 1; j < k; ++j) {
      c[static_cast<size_t>(j)] = c[static_cast<size_t>(j - 1)] + 1;
    }
  }
  return result;
}

} // namespace

// ---------------------------------------------------------------------------
// CoalitionFormationAllocator implementation
// ---------------------------------------------------------------------------
CoalitionFormationAllocator::CoalitionFormationAllocator(int max_coalition_size)
    : TaskAllocator(), max_coalition_size_(max_coalition_size) {
  this->add_ros_parameters({
      {"max_coalition_size", 3, this->max_coalition_size_},
  });
}

std::vector<TeamAllocation> CoalitionFormationAllocator::allocate(
    const std::vector<std::string> &robots,
    const std::vector<omni_plan::pddl::Predicate> &goals,
    const omni_plan::pddl::Problem &problem,
    const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        &actions) const {
  const int N = static_cast<int>(robots.size());
  const int M = static_cast<int>(goals.size());

  std::vector<TeamAllocation> result;
  if (N == 0 || M == 0) {
    return result;
  }

  // ── Pre-compute per-robot ground actions ────────────────────────────────
  constexpr std::size_t kMaxPerAction = 2048;
  const std::set<std::string> all_robot_names(robots.begin(), robots.end());

  std::vector<std::vector<GroundAction>> per_robot_acts(static_cast<size_t>(N));
  for (int i = 0; i < N; ++i) {
    per_robot_acts[static_cast<size_t>(i)] =
        ground_actions_for_robot(robots[static_cast<size_t>(i)],
                                 problem.get_objects(), actions, kMaxPerAction);
  }

  // ── Phase 1: classify each goal via per-robot relaxed reachability ───────
  // A goal is SR if at least one robot can achieve it alone (h_add < INF).
  // A goal is MR if no single robot can achieve it alone.
  const int kInf = std::numeric_limits<int>::max() / 2;

  // can_achieve[i][j] = true if robot i alone can reach goal j.
  std::vector<std::vector<bool>> can_achieve(
      static_cast<size_t>(N), std::vector<bool>(static_cast<size_t>(M), false));

  for (int i = 0; i < N; ++i) {
    const std::set<std::string> solo{robots[static_cast<size_t>(i)]};
    const auto solo_facts =
        filter_facts_for_coalition(problem.get_facts(), all_robot_names, solo);
    const auto costs = compute_relaxed_costs(
        solo_facts, per_robot_acts[static_cast<size_t>(i)]);

    for (int j = 0; j < M; ++j) {
      const auto &goal = goals[static_cast<size_t>(j)];
      const std::string gk = fact_key(goal.get_name(), goal.get_args());
      auto it = costs.find(gk);
      can_achieve[static_cast<size_t>(i)][static_cast<size_t>(j)] =
          (it != costs.end() && it->second < kInf);
    }
  }

  // MR goals: no single robot can achieve them alone.
  std::vector<bool> is_mr_goal(static_cast<size_t>(M), false);
  for (int j = 0; j < M; ++j) {
    bool any_can = false;
    for (int i = 0; i < N; ++i) {
      if (can_achieve[static_cast<size_t>(i)][static_cast<size_t>(j)]) {
        any_can = true;
        break;
      }
    }
    is_mr_goal[static_cast<size_t>(j)] = !any_can;
  }

  // ── Build BFS adjacency graph ────────────────────────────────────────────
  // Uses the shared sum-BFS from allocation_utils.hpp with robot-position
  // exclusion, consistent with CbbaAllocator and GreedyAuctionAllocator.
  const auto adj = build_object_adjacency(problem.get_facts());
  const auto robot_pos_args =
      build_robot_pos_args(problem.get_facts(), all_robot_names);

  // BFS distance matrix D[i][j]: robot i → goal j.
  // Uses INT_MAX/2 as the unreachable sentinel so that even sum-BFS values
  // (which can exceed N*M+1 for many-target goals) are stored correctly.
  const int bfs_unreachable = std::numeric_limits<int>::max() / 2;
  std::vector<std::vector<int>> D(
      static_cast<size_t>(N),
      std::vector<int>(static_cast<size_t>(M), bfs_unreachable));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      const int d = compute_bfs_distance(robots[static_cast<size_t>(i)],
                                         goals[static_cast<size_t>(j)], adj,
                                         robot_pos_args);
      if (d < bfs_unreachable) {
        D[static_cast<size_t>(i)][static_cast<size_t>(j)] = d;
      }
    }
  }

  // ── Phase 2: coalition formation for multi-robot goals ──────────────────
  // 'available[i]' tracks whether robot i is still free to be assigned solo.
  // Robots committed to a coalition are removed from the pool.
  std::vector<bool> available(static_cast<size_t>(N), true);
  std::vector<bool> goal_assigned(static_cast<size_t>(M), false);

  // Map from canonical team-name key to index in result.
  std::unordered_map<std::string, int> team_key_to_idx;

  // Collect MR goals.
  std::vector<int> mr_goal_order;
  for (int j = 0; j < M; ++j) {
    if (is_mr_goal[static_cast<size_t>(j)]) {
      mr_goal_order.push_back(j);
    }
  }

  for (int j : mr_goal_order) {
    // Collect indices of currently available robots.
    std::vector<int> avail_idx;
    for (int i = 0; i < N; ++i) {
      if (available[static_cast<size_t>(i)]) {
        avail_idx.push_back(i);
      }
    }

    const int A = static_cast<int>(avail_idx.size());
    if (A < 2) {
      // Fewer than 2 available robots: fall back to Phase 3.
      continue;
    }

    // Try subset sizes K = 2, 3, … up to max_coalition_size, picking the
    // smallest K for which a feasible coalition exists.  Among feasible
    // subsets of the same K, choose the one with minimum combined BFS
    // distance.
    std::vector<int> best_subset;
    int best_combined_dist = std::numeric_limits<int>::max();
    bool found = false;

    const int max_K = std::min(max_coalition_size_, A);
    for (int K = 2; K <= max_K && !found; ++K) {
      const auto subsets = combinations(A, K);

      for (const auto &subset_local : subsets) {
        // Build coalition set for fact filtering.
        std::set<std::string> coalition_names;
        std::vector<int> coalition_robot_indices;
        for (int idx_in_avail : subset_local) {
          const int ri = avail_idx[static_cast<size_t>(idx_in_avail)];
          coalition_names.insert(robots[static_cast<size_t>(ri)]);
          coalition_robot_indices.push_back(ri);
        }

        // Union per-robot ground actions for this coalition.
        std::vector<GroundAction> combined_acts;
        for (int ri : coalition_robot_indices) {
          const auto &r_acts = per_robot_acts[static_cast<size_t>(ri)];
          combined_acts.insert(combined_acts.end(), r_acts.begin(),
                               r_acts.end());
        }

        // Filter initial facts for the coalition.
        const auto coalition_facts = filter_facts_for_coalition(
            problem.get_facts(), all_robot_names, coalition_names);

        // Relaxed reachability check.
        const auto costs =
            compute_relaxed_costs(coalition_facts, combined_acts);
        const auto &goal = goals[static_cast<size_t>(j)];
        const std::string gk = fact_key(goal.get_name(), goal.get_args());
        auto cost_it = costs.find(gk);
        if (cost_it == costs.end() || cost_it->second >= kInf) {
          continue; // Not reachable by this coalition.
        }

        // Reachable: compute combined BFS distance as tiebreaker.
        // Guard against overflow when a coalition member has an unreachable
        // BFS distance (spatially disconnected but reachable by planning).
        int combined_dist = 0;
        for (int ri : coalition_robot_indices) {
          const int d = D[static_cast<size_t>(ri)][static_cast<size_t>(j)];
          if (d >= bfs_unreachable || combined_dist > bfs_unreachable - d) {
            combined_dist = bfs_unreachable;
            break;
          }
          combined_dist += d;
        }

        if (combined_dist < best_combined_dist) {
          best_combined_dist = combined_dist;
          best_subset = subset_local;
          found = true; // At least one feasible at this K.
        }
      }
      // If we found at least one feasible subset at this K, stop searching
      // larger K values (prefer smaller coalitions).
    }

    if (!found) {
      // No feasible coalition up to max_coalition_size: fall back to Phase 3.
      continue;
    }

    // Build the winning team from best_subset.
    std::vector<int> team_robot_indices;
    std::vector<std::string> team_robot_names;
    for (int idx_in_avail : best_subset) {
      const int ri = avail_idx[static_cast<size_t>(idx_in_avail)];
      team_robot_indices.push_back(ri);
      team_robot_names.push_back(robots[static_cast<size_t>(ri)]);
    }
    std::sort(team_robot_names.begin(), team_robot_names.end());
    std::sort(team_robot_indices.begin(), team_robot_indices.end());

    // Canonical key: comma-joined sorted names (used to merge goals into the
    // same TeamAllocation entry if the same coalition is selected twice).
    std::string key;
    for (size_t k = 0; k < team_robot_names.size(); ++k) {
      if (k > 0) {
        key += ',';
      }
      key += team_robot_names[k];
    }

    int entry_idx;
    auto it = team_key_to_idx.find(key);
    if (it != team_key_to_idx.end()) {
      entry_idx = it->second;
    } else {
      entry_idx = static_cast<int>(result.size());
      team_key_to_idx[key] = entry_idx;
      result.push_back(TeamAllocation{team_robot_names, {}});
      // Mark team members as committed.
      for (int ri : team_robot_indices) {
        available[static_cast<size_t>(ri)] = false;
      }
    }

    result[static_cast<size_t>(entry_idx)].goal_indices.push_back(j);
    goal_assigned[static_cast<size_t>(j)] = true;
  }

  // ── Phase 3: greedy BFS auction for remaining goals ─────────────────────
  // Collect remaining available robots.
  std::vector<int> solo_robots;
  std::vector<int> solo_result_idx(static_cast<size_t>(N), -1);
  for (int i = 0; i < N; ++i) {
    if (available[static_cast<size_t>(i)]) {
      solo_robots.push_back(i);
    }
  }

  if (solo_robots.empty()) {
    // All robots committed to coalitions: remaining SR goals have no owner.
    // Assign them round-robin to existing teams (load balance).
    int team_count = static_cast<int>(result.size());
    if (team_count == 0) {
      return result;
    }
    int t = 0;
    for (int j = 0; j < M; ++j) {
      if (!goal_assigned[static_cast<size_t>(j)]) {
        result[static_cast<size_t>(t % team_count)].goal_indices.push_back(j);
        ++t;
      }
    }
    return result;
  }

  // Compute load for solo robots (starts at 0).
  const int SN = static_cast<int>(solo_robots.size());
  std::vector<int> load(static_cast<size_t>(SN), 0);

  // Compute max finite BFS distance for load coefficient.
  int max_finite_dist = 0;
  for (int si = 0; si < SN; ++si) {
    const int ri = solo_robots[static_cast<size_t>(si)];
    for (int j = 0; j < M; ++j) {
      if (!goal_assigned[static_cast<size_t>(j)]) {
        const int d = D[static_cast<size_t>(ri)][static_cast<size_t>(j)];
        if (d < bfs_unreachable && d > max_finite_dist) {
          max_finite_dist = d;
        }
      }
    }
  }
  const int load_coeff = max_finite_dist + 1;

  // A capable robot (one that can achieve the goal via relaxed planning) is
  // preferred over an incapable one regardless of BFS distance.  The bonus
  // must satisfy: bonus - max_finite_dist - load_coeff * M > 0
  // (worst capable score) > 0 (best incapable score = -0 - 0).
  const int capability_bonus = max_finite_dist + load_coeff * M + 1;

  // Count remaining goals.
  int remaining = 0;
  for (int j = 0; j < M; ++j) {
    if (!goal_assigned[static_cast<size_t>(j)]) {
      ++remaining;
    }
  }

  for (int step = 0; step < remaining; ++step) {
    int best_si = -1;
    int best_j = -1;
    int best_score = std::numeric_limits<int>::min();

    for (int j = 0; j < M; ++j) {
      if (goal_assigned[static_cast<size_t>(j)]) {
        continue;
      }
      for (int si = 0; si < SN; ++si) {
        const int ri = solo_robots[static_cast<size_t>(si)];
        const int can_bonus =
            can_achieve[static_cast<size_t>(ri)][static_cast<size_t>(j)]
                ? capability_bonus
                : 0;
        const int score = can_bonus -
                          D[static_cast<size_t>(ri)][static_cast<size_t>(j)] -
                          load_coeff * load[static_cast<size_t>(si)];
        if (score > best_score) {
          best_score = score;
          best_si = si;
          best_j = j;
        }
      }
    }

    if (best_j >= 0 && best_si >= 0) {
      if (solo_result_idx[static_cast<size_t>(best_si)] < 0) {
        solo_result_idx[static_cast<size_t>(best_si)] =
            static_cast<int>(result.size());
        result.push_back(
            TeamAllocation{{robots[static_cast<size_t>(
                               solo_robots[static_cast<size_t>(best_si)])]},
                           {}});
      }
      result[static_cast<size_t>(solo_result_idx[static_cast<size_t>(best_si)])]
          .goal_indices.push_back(best_j);
      goal_assigned[static_cast<size_t>(best_j)] = true;
      ++load[static_cast<size_t>(best_si)];
    }
  }

  return result;
}

} // namespace omni_plan_mrta

PLUGINLIB_EXPORT_CLASS(omni_plan_mrta::CoalitionFormationAllocator,
                       omni_plan_mrta::TaskAllocator)
