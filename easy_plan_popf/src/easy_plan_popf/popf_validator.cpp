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

#include <fstream>
#include <iomanip>
#include <string>

#include "easy_plan_popf/popf_validator.hpp"

using namespace easy_plan_popf;

PopfValidator::PopfValidator() : PlanValidator() {}

std::string PopfValidator::parse_pddl(const easy_plan::pddl::Plan &plan) const {

  std::ostringstream oss;

  for (size_t i = 0; i < plan.size(); ++i) {
    auto [action, params] = plan.get_action_with_params(i);
    double start_time = i * 20.0;
    oss << std::fixed << std::setprecision(3) << start_time << ": ("
        << action->get_name();
    for (const auto &param : params) {
      oss << " " << param;
    }
    oss << ")  [10.000]\n";
  }

  return oss.str();
}

bool PopfValidator::validate_plan(const std::string &domain_path,
                                  const std::string &problem_path,
                                  const std::string &plan_path) const {

  // Run POPF validator - redirect stderr to stdout to capture all output
  std::string command = "ros2 run popf validate " + std::string(domain_path) +
                        " " + std::string(problem_path) + " " +
                        std::string(plan_path) + " 2>&1";

  FILE *pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return false;
  }

  // Read and discard output to prevent broken pipe errors
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    // Optionally log output for debugging
    // std::cout << buffer;
  }

  int status = pclose(pipe);
  return status == 0;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PopfValidator, easy_plan::PlanValidator)