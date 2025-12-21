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

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "easy_plan/planner.hpp"

using namespace easy_plan;

Planner::Planner() : ParameterLoader("planner") {}

pddl::Plan Planner::generate_plan(
    const pddl::Domain &domain, const pddl::Problem &problem,
    std::map<std::string, std::shared_ptr<pddl::Action>> actions) const {

  pddl::Plan plan;

  // Save domain to temporary file
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::string domain_file = temp_dir.string() + "/domain.pddl";
  std::ofstream domain_out(domain_file);
  domain_out << domain.to_pddl();
  domain_out.close();

  // Save problem to temporary file
  std::string problem_file = temp_dir.string() + "/problem.pddl";
  std::ofstream problem_out(problem_file);
  problem_out << problem.to_pddl();
  problem_out.close();

  std::string str_plan = this->generate_plan(domain_file, problem_file);

  unlink(domain_file.c_str());
  unlink(problem_file.c_str());

  plan.set_has_solution(this->has_solution(str_plan) && !str_plan.empty());

  if (!plan.has_solution()) {
    return plan;
  }

  std::vector<std::string> lines = this->get_lines_with_actions(str_plan);
  for (const auto &line : lines) {
    auto [action_name, parameters] = this->parse_action_line(line);
    plan.add_action(actions.at(action_name), parameters);
  }

  return plan;
}

std::pair<std::string, std::vector<std::string>>
Planner::parse_action_line(std::string line) const {
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
