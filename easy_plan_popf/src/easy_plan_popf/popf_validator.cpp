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
  char domain_file[] = "/tmp/easy_plan_domainXXXXXX";
  int fd1 = mkstemp(domain_file);
  if (fd1 == -1) {
    return false;
  }
  close(fd1);

  std::ofstream domain_out(domain_file);
  domain_out << domain;
  domain_out.close();

  // Save problem to temporary file
  char problem_file[] = "/tmp/easy_plan_problemXXXXXX";
  int fd2 = mkstemp(problem_file);
  if (fd2 == -1) {
    unlink(domain_file);
    return false;
  }
  close(fd2);

  // Save plan to temporary file
  std::ofstream problem_out(problem_file);
  problem_out << problem;
  problem_out.close();

  char plan_file[] = "/tmp/easy_plan_planXXXXXX";
  int fd3 = mkstemp(plan_file);
  if (fd3 == -1) {
    unlink(domain_file);
    unlink(problem_file);
    return false;
  }
  close(fd3);

  std::ofstream plan_out(plan_file);
  plan_out << this->plan_to_string(plan);
  plan_out.close();

  // Run POPF planner
  std::string command = "ros2 run popf validate " + std::string(domain_file) +
                        " " + std::string(problem_file) + " " +
                        std::string(plan_file);
  FILE *pipe = popen(command.c_str(), "r");
  if (!pipe) {
    unlink(domain_file);
    unlink(problem_file);
    unlink(plan_file);
    return false;
  }

  std::string output;
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  int status = pclose(pipe);
  unlink(domain_file);
  unlink(problem_file);
  unlink(plan_file);

  return status == 0;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PopfValidator, easy_plan::PlanValidator)