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

#ifndef OMNI_PLAN_LPG__LPG_PLANNER_HPP_
#define OMNI_PLAN_LPG__LPG_PLANNER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/planner.hpp"

namespace omni_plan_lpg {

/**
 * @class LpgPlanner
 * @brief Planner implementation using the LPG (Local search for Planning
 * Graphs) algorithm.
 * @details This class provides automated planning capabilities using the LPG
 * algorithm, which is a local-search-based planner for PDDL temporal and
 * metric planning domains. LPG uses a stochastic local search on planning
 * graphs and supports both classical and temporal planning.
 */
class LpgPlanner : public omni_plan::Planner {
public:
  /**
   * @brief Default constructor for LpgPlanner.
   * @details Initializes the LPG planner with default settings and prepares
   * it for solving planning problems.
   */
  LpgPlanner();

  /**
   * @brief Generates a plan from PDDL domain and problem file paths.
   * @param domain_path The file path to the PDDL domain definition.
   * @param problem_path The file path to the PDDL problem definition.
   * @return A string representing the generated plan output.
   */
  std::string generate_plan(const std::string domain_path,
                            const std::string problem_path) const override;

  using Planner::generate_plan;

  /**
   * @brief Checks if the generated plan output indicates a valid solution.
   * @param plan_output The complete planner output as a string.
   * @return True if the plan represents a valid solution, false otherwise.
   */
  bool has_solution(const std::string &plan_output) const override;

  /**
   * @brief Extracts lines containing actions from the LPG plan output.
   * @details LPG prints a header line with the format
   * "Time: (ACTION) [action Duration; action Cost]" which must be excluded.
   * Only lines whose content before the first ':' is a valid timestamp number
   * are returned.
   * @param plan_str The complete plan output as a string.
   * @return A vector of strings, each representing an action line.
   */
  std::vector<std::string>
  get_lines_with_actions(const std::string &plan_str) const override;

  /**
   * @brief Parses the duration from a LPG plan output line.
   * @details LPG uses the format "[D:10.000; C:1.000]" for durations, so the
   * "D:" prefix must be stripped before converting to a float.
   * @param line A line from the planner's output.
   * @return The duration as a float, or 0.0 if not found.
   */
  float parse_duration(const std::string &line) const override;

  /**
   * @brief Parses an action line from the LPG plan output.
   * @details LPG outputs action names and parameters in uppercase. This
   * override converts the content to lowercase before parsing so that action
   * names match the lowercase keys used in the actions map.
   * @param line A line from the planner's output representing an action.
   * @return A pair of the (lowercase) action name and its parameters.
   */
  std::pair<std::string, std::vector<std::string>>
  parse_action_line(std::string line) const override;

private:
  /// @brief Number of solutions to compute.
  int num_solutions_;
  /// @brief Heuristic function: 1 or 2 (default 1).
  int heuristic_;
  /// @brief Maximum number of restarts (default 50).
  int restarts_;
  /// @brief Number of steps in the first run of the local search (default 500).
  int search_steps_;
  /// @brief Noise factor for Walkplan, range [0, 1] (default 0.1).
  double noise_;
  /// @brief Switch off best-first search.
  bool nobestfirst_;
  /// @brief Immediately run best-first search.
  bool onlybestfirst_;
  /// @brief Random seed (0 means not set).
  int seed_;
  /// @brief Inconsistency selection method (default 2).
  int i_choice_;
  /// @brief Maximum CPU time in seconds (0 means unlimited).
  int cputime_;
  /// @brief Take overlapping of actions into account during neighbourhood
  /// evaluation.
  bool advanced_time_;
};

} // namespace omni_plan_lpg
#endif // OMNI_PLAN_LPG__LPG_PLANNER_HPP_
