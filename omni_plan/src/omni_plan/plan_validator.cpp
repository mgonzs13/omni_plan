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

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "omni_plan/plan_validator.hpp"
#include "omni_plan/utils/parameter_loader.hpp"

using namespace omni_plan;

namespace {

struct TempFileGuard {
  const char *path;
  TempFileGuard(const char *p) : path(p) {}
  ~TempFileGuard() {
    if (path) {
      std::remove(path);
    }
  }
  TempFileGuard(const TempFileGuard &) = delete;
  TempFileGuard &operator=(const TempFileGuard &) = delete;
};

} // namespace

PlanValidator::PlanValidator() : utils::ParameterLoader("plan_validator") {}

bool PlanValidator::validate_plan(const pddl::Domain &domain,
                                  const pddl::Problem &problem,
                                  const pddl::Plan &plan) const {

  static std::atomic<int> call_counter{0};
  const std::string suffix =
      "_" + std::to_string(getpid()) + "_" +
      std::to_string(call_counter.fetch_add(1, std::memory_order_relaxed));

  // Save domain to temporary file
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::string domain_file = temp_dir.string() + "/domain" + suffix + ".pddl";
  std::ofstream domain_out(domain_file);
  domain_out << domain.to_pddl();
  domain_out.close();
  TempFileGuard domain_guard(domain_file.c_str());

  // Save problem to temporary file
  std::string problem_file = temp_dir.string() + "/problem" + suffix + ".pddl";
  std::ofstream problem_out(problem_file);
  problem_out << problem.to_pddl();
  problem_out.close();
  TempFileGuard problem_guard(problem_file.c_str());

  // Save plan to temporary file
  std::string plan_file = temp_dir.string() + "/plan" + suffix + ".pddl";
  std::ofstream plan_out(plan_file);
  plan_out << plan.to_pddl();
  plan_out.close();
  TempFileGuard plan_guard(plan_file.c_str());

  return this->validate_plan(domain_file, problem_file, plan_file);
}
