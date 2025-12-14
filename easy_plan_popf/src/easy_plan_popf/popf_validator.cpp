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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "easy_plan_popf/popf_validator.hpp"

using namespace easy_plan_popf;

PopfValidator::PopfValidator() : PlanValidator() {}

std::string PopfValidator::plan_to_string(easy_plan::Plan plan) const {
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

bool PopfValidator::validate_plan(const std::string &domain,
                                  const std::string &problem,
                                  easy_plan::Plan plan) const {

  // Save domain to temporary file
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::string domain_file = temp_dir.string() + "/domain.pddl";
  std::ofstream domain_out(domain_file);
  domain_out << domain;
  domain_out.close();

  // Save problem to temporary file
  std::string problem_file = temp_dir.string() + "/problem.pddl";
  std::ofstream problem_out(problem_file);
  problem_out << problem;
  problem_out.close();

  // Save plan to temporary file
  std::string plan_file = temp_dir.string() + "/plan.txt";
  std::ofstream plan_out(plan_file);
  plan_out << this->plan_to_string(plan);
  plan_out.close();

  // Run POPF planner
  std::string output_file = temp_dir.string() + "/output.txt";
  std::string command = "ros2 run popf validate " + std::string(domain_file) +
                        " " + std::string(problem_file) + " " +
                        std::string(plan_file) + " > " + output_file;
  int status = std::system(command.c_str());

  std::ifstream output_in(output_file);
  std::string output((std::istreambuf_iterator<char>(output_in)),
                     std::istreambuf_iterator<char>());

  unlink(domain_file.c_str());
  unlink(problem_file.c_str());
  unlink(plan_file.c_str());
  unlink(output_file.c_str());

  return status == 0;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PopfValidator, easy_plan::PlanValidator)