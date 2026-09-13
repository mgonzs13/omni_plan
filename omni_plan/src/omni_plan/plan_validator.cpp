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
#include <string>

#include "omni_plan/plan_validator.hpp"
#include "omni_plan/utils/parameter_loader.hpp"
#include "omni_plan/utils/temp_file_guard.hpp"

using namespace omni_plan;

PlanValidator::PlanValidator() : utils::ParameterLoader("plan_validator") {}

bool PlanValidator::validate_plan(const pddl::Domain &domain,
                                  const pddl::Problem &problem,
                                  const pddl::Plan &plan) const {

  // Create a private, per-call directory so concurrent validations (and
  // Planner) can never collide or follow a pre-existing symlink in the shared
  // temporary directory.
  std::string temp_dir = utils::create_private_temp_dir("omni_plan_validator");
  if (temp_dir.empty()) {
    std::cerr << "[plan_validator] Failed to create a temporary directory"
              << std::endl;
    return false;
  }
  utils::TempDirGuard dir_guard(temp_dir);

  // Save domain to temporary file (owner-only permissions)
  std::string domain_file = temp_dir + "/domain.pddl";
  if (!utils::write_private_file(domain_file, domain.to_pddl())) {
    std::cerr << "[plan_validator] Failed to write domain PDDL file: "
              << domain_file << std::endl;
    return false;
  }
  utils::TempFileGuard domain_guard(domain_file);

  // Save problem to temporary file (owner-only permissions)
  std::string problem_file = temp_dir + "/problem.pddl";
  if (!utils::write_private_file(problem_file, problem.to_pddl())) {
    std::cerr << "[plan_validator] Failed to write problem PDDL file: "
              << problem_file << std::endl;
    return false;
  }
  utils::TempFileGuard problem_guard(problem_file);

  // Save plan to temporary file (owner-only permissions)
  std::string plan_file = temp_dir + "/plan.pddl";
  if (!utils::write_private_file(plan_file, plan.to_pddl())) {
    std::cerr << "[plan_validator] Failed to write plan PDDL file: "
              << plan_file << std::endl;
    return false;
  }
  utils::TempFileGuard plan_guard(plan_file);

  return this->validate_plan(domain_file, problem_file, plan_file);
}
