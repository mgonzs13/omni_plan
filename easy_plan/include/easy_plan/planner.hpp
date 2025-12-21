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

#ifndef EASY_PLAN__PLANNER_HPP_
#define EASY_PLAN__PLANNER_HPP_

#include <map>
#include <memory>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl/plan.hpp"

namespace easy_plan {

/**
 * @class Planner
 * @brief Base class for automated planning algorithms.
 * @details This abstract base class defines the interface for planning
 * components that solve PDDL planning problems. Planners implement various
 * search algorithms to find sequences of actions that achieve the goals defined
 * in the problem.
 */
class Planner {
public:
  /**
   * @brief Default constructor.
   */
  Planner() = default;

  /**
   * @brief Default destructor.
   */
  virtual ~Planner() = default;

  /**
   * @brief Generates a plan to solve a PDDL planning problem.
   * @details This pure virtual method must be implemented by derived classes to
   * provide the specific planning algorithm. It takes PDDL domain and problem
   * definitions and a map of available actions, then returns a plan that solves
   * the problem or indicates that no solution was found.
   * @param domain The PDDL domain definition as a string.
   * @param problem The PDDL problem definition as a string.
   * @param actions A map of action names to action objects available for
   * planning.
   * @return A Plan object containing the solution or indicating no solution
   * found.
   */
  virtual pddl::Plan
  get_plan(const std::string &domain, const std::string &problem,
           std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>>
               actions) const = 0;
};

} // namespace easy_plan
#endif // EASY_PLAN__PLANNER_HPP_