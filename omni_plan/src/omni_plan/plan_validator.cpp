// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
