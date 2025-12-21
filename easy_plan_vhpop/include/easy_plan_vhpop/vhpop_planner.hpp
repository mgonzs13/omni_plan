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

#ifndef EASY_PLAN__VHPOP_PLANNER_HPP_
#define EASY_PLAN__VHPOP_PLANNER_HPP_

#include <memory>
#include <string>

#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/planner.hpp"

namespace easy_plan_vhpop {

/**
 * @class VhpopPlanner
 * @brief Planner implementation using the VHPOP (Versatile Heuristic Partial
 * Order Planner) algorithm.
 * @details This class provides automated planning capabilities using the VHPOP
 * algorithm, which is particularly effective for temporal planning domains with
 * durative actions. VHPOP can handle complex planning problems involving
 * concurrent actions and temporal constraints.
 */
class VhpopPlanner : public easy_plan::Planner {
public:
  /**
   * @brief Default constructor for VhpopPlanner.
   * @details Initializes the VHPOP planner with default settings and prepares
   * it for solving planning problems.
   */
  VhpopPlanner();

  /**
   * @brief Generates a plan using the VHPOP algorithm.
   * @details Implements the planning algorithm to find a sequence of actions
   * that solves the given PDDL planning problem. Uses partial order planning
   * techniques combined with first-order logic reasoning to handle complex
   * temporal domains.
   * @param domain The PDDL domain definition as a string.
   * @param problem The PDDL problem definition as a string.
   * @param actions A map of action names to action objects available for
   * planning.
   * @return A Plan object containing the solution or indicating no solution was
   * found.
   */
  easy_plan::pddl::Plan
  get_plan(const std::string &domain, const std::string &problem,
           std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>>
               actions) const override;

private:
};

} // namespace easy_plan_vhpop
#endif // EASY_PLAN__VHPOP_PLANNER_HPP_