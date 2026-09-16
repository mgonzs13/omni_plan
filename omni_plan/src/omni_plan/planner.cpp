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

#include <iostream>
#include <sstream>
#include <string>

#include "omni_plan/planner.hpp"
#include "omni_plan/utils/parameter_loader.hpp"
#include "omni_plan/utils/temp_file_guard.hpp"

using namespace omni_plan;

Planner::Planner() : utils::ParameterLoader("planner") {}

pddl::Plan Planner::generate_plan(const pddl::Domain &domain,
                                  const pddl::Problem &problem) const {

  // Create a private, per-call directory so concurrent planning calls (and
  // PlanValidator) can never collide or follow a pre-existing symlink in the
  // shared temporary directory.
  std::string temp_dir = utils::create_private_temp_dir("omni_plan_planner");
  if (temp_dir.empty()) {
    std::cerr << "[planner] Failed to create a temporary directory"
              << std::endl;
    return pddl::Plan();
  }
  utils::TempDirGuard dir_guard(temp_dir);

  // Save domain to temporary file (owner-only permissions)
  std::string domain_file = temp_dir + "/domain.pddl";
  if (!utils::write_private_file(domain_file, domain.to_pddl())) {
    std::cerr << "[planner] Failed to write domain PDDL file: " << domain_file
              << std::endl;
    return pddl::Plan();
  }
  utils::TempFileGuard domain_guard(domain_file);

  // Save problem to temporary file (owner-only permissions)
  std::string problem_file = temp_dir + "/problem.pddl";
  if (!utils::write_private_file(problem_file, problem.to_pddl())) {
    std::cerr << "[planner] Failed to write problem PDDL file: " << problem_file
              << std::endl;
    return pddl::Plan();
  }
  utils::TempFileGuard problem_guard(problem_file);

  std::string str_plan = this->generate_plan(domain_file, problem_file);

  pddl::Plan plan = this->parse_plan(domain, str_plan);

  // A planner success without actions is a valid no-op plan when the problem
  // has no goals: the goals may have been satisfied (or removed) while the
  // planning was starting. Without this, the plan state aborts and the state
  // machine re-enters the planning generation without checking the goals
  // again, which produces an infinite reasoning loop.
  if (!plan.has_solution() && problem.get_goals().empty() &&
      this->has_solution(str_plan) && !str_plan.empty()) {
    plan.set_has_solution(true);
  }

  return plan;
}

pddl::Plan Planner::parse_plan(const pddl::Domain &domain,
                               const std::string &str_plan) const {

  pddl::Plan plan;

  plan.set_raw_output(str_plan);
  plan.set_has_solution(this->has_solution(str_plan) && !str_plan.empty());

  if (!plan.has_solution()) {
    return plan;
  }

  // Parse the plan output to extract actions and their parameters
  const auto &actions = domain.get_actions();
  std::vector<std::string> lines = this->get_lines_with_actions(str_plan);

  for (const auto &line : lines) {
    auto [action_name, parameters] = this->parse_action_line(line);

    if (action_name.empty()) {
      continue;
    }

    float start_time = this->parse_start_time(line);
    auto it = actions.find(action_name);
    if (it == actions.end()) {
      std::cerr << "[planner] Unknown action '" << action_name
                << "' in plan output; marking plan as invalid" << std::endl;
      plan.set_has_solution(false);
      break;
    }
    plan.add_action(it->second, parameters, start_time);
  }

  // A success marker without a single resolvable action is not a valid plan.
  if (plan.has_solution() && plan.size() == 0) {
    std::cerr << "[planner] Planner reported a solution but no actions were "
                 "parsed; marking plan as invalid"
              << std::endl;
    plan.set_has_solution(false);
  }

  return plan;
}

std::pair<std::string, std::vector<std::string>>
Planner::parse_action_line(const std::string &line) const {
  size_t start = line.find('(');
  size_t end = line.find(')', start);
  if (start == std::string::npos || end == std::string::npos) {
    return {"", {}};
  }

  std::string action_part = line.substr(start + 1, end - start - 1);
  std::vector<std::string> parts;
  std::stringstream ss(action_part);
  std::string token;

  while (ss >> token) {
    parts.push_back(token);
  }

  if (parts.empty()) {
    return {"", {}};
  }

  std::string action_name = parts[0];
  std::vector<std::string> parameters(parts.begin() + 1, parts.end());
  return {action_name, parameters};
}

std::vector<std::string>
Planner::get_lines_with_actions(const std::string &plan_str) const {
  std::vector<std::string> pddl_action_list;
  std::stringstream ss(plan_str);
  std::string line;

  while (std::getline(ss, line)) {
    if (line.find('(') != std::string::npos &&
        line.find(')') != std::string::npos &&
        line.find('[') != std::string::npos &&
        line.find(']') != std::string::npos) {
      pddl_action_list.push_back(line);
    }
  }

  return pddl_action_list;
}

float Planner::parse_start_time(const std::string &line) const {
  size_t colon_pos = line.find(':');
  if (colon_pos == std::string::npos) {
    return 0.0f;
  }

  try {
    return std::stof(line.substr(0, colon_pos));
  } catch (...) {
    return 0.0f;
  }
}
