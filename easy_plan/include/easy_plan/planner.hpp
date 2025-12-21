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
#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/utils/parameter_loader.hpp"

namespace easy_plan {

/**
 * @class Planner
 * @brief Base class for automated planning algorithms.
 * @details This abstract base class defines the interface for planning
 * components that solve PDDL planning problems. Planners implement various
 * search algorithms to find sequences of actions that achieve the goals defined
 * in the problem.
 */
class Planner : public utils::ParameterLoader {
public:
  /**
   * @brief Default constructor.
   */
  Planner();

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
  pddl::Plan generate_plan(
      const pddl::Domain &domain, const pddl::Problem &problem,
      std::map<std::string, std::shared_ptr<pddl::Action>> actions) const;

protected:
  /**
   * @brief Generates a plan from PDDL domain and problem file paths.
   * @details This pure virtual method must be implemented by derived classes to
   * read PDDL domain and problem definitions from files, then generate a plan
   * to solve the problem.
   * @param domain_path The file path to the PDDL domain definition.
   * @param problem_path The file path to the PDDL problem definition.
   * @return A string representing the generated plan in PDDL format.
   */
  virtual std::string generate_plan(const std::string domain_path,
                                    const std::string problem_path) const = 0;

  /**
   * @brief Checks if the generated plan indicates a valid solution.
   * @details This pure virtual method must be implemented by derived classes to
   * analyze the plan output and determine if it represents a valid solution to
   * the planning problem.
   * @param plan_str The generated plan as a string.
   * @return True if the plan represents a valid solution, false otherwise.
   */
  virtual bool has_solution(const std::string &plan_str) const = 0;

  /**
   * @brief Parses a line from the plan output to extract the action and its
   * parameters.
   * @details This pure virtual method must be implemented by derived classes to
   * interpret lines from the planner's output and extract the action name and
   * parameters.
   * @param line A line from the planner's output representing an action.
   * @return A pair containing the action name and a string of its parameters.
   */
  virtual std::pair<std::string, std::vector<std::string>>
  parse_action_line(std::string line) const;

  /**
   * @brief Extracts lines containing actions from the plan output.
   * @details This pure virtual method must be implemented by derived classes to
   * filter and return only the lines from the planner's output that represent
   * actions in the generated plan.
   * @param plan_str The complete plan output as a string.
   * @return A vector of strings, each representing an action line from the
   * plan.
   */
  virtual std::vector<std::string>
  get_lines_with_actions(const std::string &plan_str) const;
};

} // namespace easy_plan
#endif // EASY_PLAN__PLANNER_HPP_