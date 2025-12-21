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

#include "easy_plan/plan_validator.hpp"

using namespace easy_plan;

PlanValidator::PlanValidator() : ParameterLoader("plan_validator") {}

bool PlanValidator::validate_plan(const pddl::Domain &domain,
                                  const pddl::Problem &problem,
                                  pddl::Plan plan) const {

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

  // Save plan to temporary file
  std::string plan_file = temp_dir.string() + "/plan.pddl";
  std::ofstream plan_out(plan_file);
  plan_out << this->parse_pddl(plan);
  plan_out.close();

  // Run plan validator
  bool is_valid = this->validate_plan(domain_file, problem_file, plan_file);

  unlink(domain_file.c_str());
  unlink(problem_file.c_str());
  unlink(plan_file.c_str());

  return is_valid;
}
