// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <sys/wait.h>

#include <cstdio>
#include <iostream>
#include <string>

#include "omni_plan/utils/package_share_path.hpp"

#include "omni_plan_colin/colin_planner.hpp"

using namespace omni_plan_colin;

namespace {

std::string shell_quote(const std::string &value) {
  std::string quoted = "'";
  for (const char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

} // namespace

ColinPlanner::ColinPlanner() : Planner() {
  // Add COLIN options as parameters
  this->add_ros_parameters(
      {{"disable_best_first", false, this->disable_best_first_},
       {"skip_ehc", false, this->skip_ehc_},
       {"standard_ehc", false, this->standard_ehc_},
       {"disable_helpful_pruning", false, this->disable_helpful_pruning_},
       {"disable_compression_safe", false, this->disable_compression_safe_},
       {"disable_tie_breaking_search", false,
        this->disable_tie_breaking_search_},
       {"full_ff_helpful", false, this->full_ff_helpful_},
       {"total_order", false, this->total_order_},
       {"timeout", 0, this->timeout_}});
}

std::string ColinPlanner::generate_plan(const std::string &domain_path,
                                        const std::string &problem_path) const {

  // Build command with options
  std::string command =
      shell_quote(omni_plan::utils::get_package_share_path("omni_plan_colin") +
                  "/bin/colin");

  if (this->disable_best_first_)
    command += " -b";
  if (this->skip_ehc_)
    command += " -E";
  if (this->standard_ehc_)
    command += " -e";
  if (this->disable_helpful_pruning_)
    command += " -h";
  if (this->disable_compression_safe_)
    command += " -k";
  if (this->disable_tie_breaking_search_)
    command += " -M";
  if (this->full_ff_helpful_)
    command += " -F";
  if (this->total_order_)
    command += " -T";

  command += " " + shell_quote(domain_path) + " " + shell_quote(problem_path);

  if (this->timeout_ > 0)
    command = "timeout -k 5 " + std::to_string(this->timeout_) + " " + command;

  // Run COLIN planner
  FILE *pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return "";
  }

  std::string output;
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  int status = pclose(pipe);
  if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "[colin] COLIN terminated abnormally" << std::endl;
    return "";
  }

  return output;
}

bool ColinPlanner::has_solution(const std::string &plan_output) const {
  return plan_output.find("Solution Found") != std::string::npos;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ColinPlanner, omni_plan::Planner)
