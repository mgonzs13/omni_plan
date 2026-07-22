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

#ifndef OMNI_PLAN_MRTA__ALLOCATORS__COALITION_FORMATION_ALLOCATOR_HPP_
#define OMNI_PLAN_MRTA__ALLOCATORS__COALITION_FORMATION_ALLOCATOR_HPP_

#include "omni_plan_mrta/task_allocator.hpp"

namespace omni_plan_mrta {

/**
 * @class CoalitionFormationAllocator
 * @brief PDDL-aware task allocator that forms robot teams for multi-robot
 * goals and assigns individual goals to single robots.
 *
 * @details This allocator implements a three-phase greedy coalition formation
 * strategy based on delete-relaxed reachability analysis:
 *
 * **Phase 1 – Goal classification via per-robot relaxed planning**
 * For each goal the allocator grounds all PDDL action templates that involve
 * each robot and runs h_add (delete-relaxed Bellman-Ford) against the subset
 * of the initial state relevant to that robot.  If any single robot can
 * achieve the goal (finite h_add cost), the goal is classified as SR
 * (*single-robot*).  If no single robot can reach the goal, it is classified
 * as MR (*multi-robot*), indicating that cooperation between robots with
 * complementary action pools is required.
 *
 * This correctly handles scenarios where no single action has multiple
 * robot-type parameters but cooperation is still required — e.g., robot_a
 * must place an object and robot_b must process it.
 *
 * **Phase 2 – Coalition formation for MR goals**
 * For each MR goal the allocator enumerates K-subsets of currently available
 * robots (K = 2, then 3, … up to max_coalition_size) and, for each subset,
 * performs a full relaxed-planning reachability check by pooling the
 * combined grounded action sets and the coalition-filtered initial state.
 * The smallest K for which a feasible coalition exists is chosen.  Among all
 * feasible subsets of that size the one with the smallest combined BFS
 * distance to the goal arguments is selected.  Committed robots are removed
 * from the solo pool.  Goals for which no feasible coalition is found fall
 * back to Phase 3.
 *
 * **Phase 3 – Greedy auction for remaining goals**
 * SR goals and MR fallbacks are assigned to the still-available robots using
 * a load-balanced BFS-distance greedy auction identical to
 * GreedyAuctionAllocator.
 *
 * ROS parameter: @c allocator.max_coalition_size (int, default 3) — caps the
 * required coalition size considered during Phase 2 to bound search.
 *
 * References:
 *   Shehory, O. & Kraus, S. (1998). Methods for task allocation via agent
 *   coalition formation. Artificial Intelligence, 101(1–2), 165–200.
 *
 *   Gerkey, B. P. & Matarić, M. J. (2004). A formal analysis and taxonomy of
 *   task allocation in multi-robot systems. The International Journal of
 *   Robotics Research, 23(9), 939–954.
 *   https://doi.org/10.1177/0278364904045564
 */
class CoalitionFormationAllocator : public TaskAllocator {
public:
  /**
   * @brief Constructs the allocator.
   * @param max_coalition_size Maximum coalition size to consider.
   *        Can also be set via the ROS 2 parameter
   *        @c allocator.max_coalition_size.
   */
  explicit CoalitionFormationAllocator(int max_coalition_size = 3);

  std::vector<TeamAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;

private:
  /// @brief Maximum coalition size to consider during Phase 2
  int max_coalition_size_;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__COALITION_FORMATION_ALLOCATOR_HPP_
