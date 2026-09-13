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

#include <sys/wait.h>

#include <cstdio>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "omni_plan/utils/package_share_path.hpp"

#include "omni_plan_vhpop/vhpop_planner.hpp"

using namespace omni_plan_vhpop;

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

bool is_one_of(const std::string &value,
               std::initializer_list<const char *> allowed) {
  for (const char *candidate : allowed) {
    if (value == candidate) {
      return true;
    }
  }
  return false;
}

bool is_safe_enum(const std::string &value) {
  static const std::string allowed = "abcdefghijklmnopqrstuvwxyz"
                                     "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                     "0123456789_+*/{}().,-";
  return !value.empty() && value.size() <= 64 &&
         value.find_first_not_of(allowed) == std::string::npos;
}

} // namespace

VhpopPlanner::VhpopPlanner() : Planner() {
  // Add VHPOP options as parameters
  this->add_ros_parameters(
      {{"action_cost", std::string("UNIT"), this->action_cost_},
       {"domain_constraints", 1, this->domain_constraints_},
       {"flaw_order", std::string("ZLIFO"), this->flaw_order_},
       {"ground_actions", false, this->ground_actions_},
       {"heuristic", std::string("S+OC"), this->heuristic_},
       {"limit", 0, this->limit_},
       {"random_open_conditions", false, this->random_open_conditions_},
       {"seed", 0, this->seed_},
       {"search_algorithm", std::string("HC"), this->search_algorithm_},
       {"time_limit", 0, this->time_limit_},
       {"tolerance", 0.01f, this->tolerance_},
       {"weight", 1.0f, this->weight_},
       {"timeout", 0, this->timeout_}});
}

std::string VhpopPlanner::generate_plan(const std::string &domain_path,
                                        const std::string &problem_path) const {

  // Build command with options
  std::string command =
      shell_quote(omni_plan::utils::get_package_share_path("omni_plan_vhpop") +
                  "/bin/vhpop");

  if (!this->action_cost_.empty()) {
    if (is_one_of(this->action_cost_, {"UNIT", "DURATION", "RELATIVE"})) {
      command += " -a " + shell_quote(this->action_cost_);
    } else {
      std::cerr << "[vhpop] Ignoring invalid action_cost parameter: "
                << this->action_cost_ << std::endl;
    }
  }
  if (this->domain_constraints_ != 1)
    command += " -d" + std::to_string(this->domain_constraints_);
  if (!this->flaw_order_.empty()) {
    if (is_safe_enum(this->flaw_order_)) {
      command += " -f " + shell_quote(this->flaw_order_);
    } else {
      std::cerr << "[vhpop] Ignoring invalid flaw_order parameter: "
                << this->flaw_order_ << std::endl;
    }
  }
  if (this->ground_actions_)
    command += " -g";
  if (!this->heuristic_.empty()) {
    if (is_safe_enum(this->heuristic_)) {
      command += " -h " + shell_quote(this->heuristic_);
    } else {
      std::cerr << "[vhpop] Ignoring invalid heuristic parameter: "
                << this->heuristic_ << std::endl;
    }
  }
  if (this->limit_ > 0)
    command += " -l " + std::to_string(limit_);
  if (this->random_open_conditions_)
    command += " -r";
  if (this->seed_ != 0)
    command += " -S " + std::to_string(this->seed_);
  if (!this->search_algorithm_.empty()) {
    if (is_one_of(this->search_algorithm_, {"IDA", "HC"})) {
      command += " -s " + shell_quote(this->search_algorithm_);
    } else {
      std::cerr << "[vhpop] Ignoring invalid search_algorithm parameter: "
                << this->search_algorithm_ << std::endl;
    }
  }
  if (this->time_limit_ > 0)
    command += " -T " + std::to_string(this->time_limit_);
  if (this->tolerance_ != 0.01f)
    command += " -t " + std::to_string(this->tolerance_);
  if (this->weight_ != 1.0f)
    command += " -w " + std::to_string(this->weight_);

  command += " " + shell_quote(domain_path) + " " + shell_quote(problem_path);

  if (this->timeout_ > 0)
    command = "timeout -k 5 " + std::to_string(this->timeout_) + " " + command;

  // Run VHPOP planner
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
    std::cerr << "[vhpop] VHPOP terminated abnormally" << std::endl;
    return "";
  }

  return output;
}

std::vector<std::string>
VhpopPlanner::get_lines_with_actions(const std::string &plan_str) const {
  std::vector<std::string> action_lines;
  std::stringstream ss(plan_str);
  std::string line;

  while (std::getline(ss, line)) {
    if (line.find('(') == std::string::npos ||
        line.find(')') == std::string::npos) {
      continue;
    }

    size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }
    try {
      std::stof(line.substr(0, colon_pos));
    } catch (...) {
      continue;
    }

    action_lines.push_back(line);
  }

  return action_lines;
}

bool VhpopPlanner::has_solution(const std::string &plan_str) const {
  return plan_str.find("Time:") != std::string::npos &&
         plan_str.find("no plan") == std::string::npos;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(VhpopPlanner, omni_plan::Planner)