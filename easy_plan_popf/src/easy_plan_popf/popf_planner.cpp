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

#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "easy_plan_popf/popf_planner.hpp"

using namespace easy_plan_popf;

PopfPlanner::PopfPlanner() : Planner() {}

easy_plan::Plan PopfPlanner::get_plan(
    const std::string &domain, const std::string &problem,
    std::map<std::string, std::shared_ptr<easy_plan::pddl::Action>> actions)
    const {

  // Create temporary files
  char domain_file[] = "/tmp/easy_plan_domain";
  int fd1 = mkstemp(domain_file);
  if (fd1 == -1) {
    return easy_plan::Plan(false);
  }
  close(fd1);

  std::ofstream domain_out(domain_file);
  domain_out << domain;
  domain_out.close();

  char problem_file[] = "/tmp/easy_plan_problem";
  int fd2 = mkstemp(problem_file);
  if (fd2 == -1) {
    unlink(domain_file);
    return easy_plan::Plan(false);
  }
  close(fd2);

  std::ofstream problem_out(problem_file);
  problem_out << problem;
  problem_out.close();

  // Run POPF planner
  std::string command = "ros2 run popf popf " + std::string(domain_file) + " " +
                        std::string(problem_file);
  FILE *pipe = popen(command.c_str(), "r");
  if (!pipe) {
    unlink(domain_file);
    unlink(problem_file);
    return easy_plan::Plan(false);
  }

  std::string output;
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  int status = pclose(pipe);
  unlink(domain_file);
  unlink(problem_file);

  if (status != 0 || output.find("Solution found") == std::string::npos) {
    return easy_plan::Plan(false);
  }

  // Parse the plan
  easy_plan::Plan plan(true);
  std::istringstream iss(output);
  std::string line;

  while (std::getline(iss, line)) {
    size_t colon_pos = line.find(": (");
    if (colon_pos != std::string::npos) {
      std::string action_part = line.substr(colon_pos + 3);
      size_t paren_pos = action_part.find(')');
      if (paren_pos != std::string::npos) {
        std::string action_str = action_part.substr(0, paren_pos);
        std::istringstream action_iss(action_str);
        std::string action_name;
        action_iss >> action_name;
        std::vector<std::string> params;
        std::string param;
        while (action_iss >> param) {
          params.push_back(param);
        }
        auto action = actions[action_name];
        plan.add_action(action, params);
      }
    }
  }

  return plan;
}