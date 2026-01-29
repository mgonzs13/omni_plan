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

#include <cstdio>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "omni_plan_vhpop/vhpop_planner.hpp"

using namespace omni_plan_vhpop;

VhpopPlanner::VhpopPlanner() : Planner() {
  // Add VHPOP options as parameters
  this->add_ros_parameters(
      {{"action_cost", 0.0f, this->action_cost_},
       {"domain_constraints", 1, this->domain_constraints_},
       {"flaw_order", std::string(""), this->flaw_order_},
       {"ground_actions", false, this->ground_actions_},
       {"heuristic", std::string(""), this->heuristic_},
       {"limit", 0, this->limit_},
       {"random_open_conditions", false, this->random_open_conditions_},
       {"seed", 0, this->seed_},
       {"search_algorithm", std::string(""), this->search_algorithm_},
       {"time_limit", 0, this->time_limit_},
       {"tolerance", 0.01f, this->tolerance_},
       {"weight", 1.0f, this->weight_}});
}

std::string VhpopPlanner::generate_plan(const std::string domain_path,
                                        const std::string problem_path) const {

  // Build command with options
  std::string command =
      ament_index_cpp::get_package_share_directory("omni_plan_vhpop") +
      "/bin/vhpop";

  if (this->action_cost_ != 0.0f)
    command += " -a " + std::to_string(this->action_cost_);
  if (this->domain_constraints_ != 1)
    command += " -d" + std::to_string(this->domain_constraints_);
  if (!this->flaw_order_.empty())
    command += " -f " + this->flaw_order_;
  if (this->ground_actions_)
    command += " -g";
  if (!this->heuristic_.empty())
    command += " -h " + heuristic_;
  if (this->limit_ > 0)
    command += " -l " + std::to_string(limit_);
  if (this->random_open_conditions_)
    command += " -r";
  if (this->seed_ != 0)
    command += " -S " + std::to_string(this->seed_);
  if (!this->search_algorithm_.empty())
    command += " -s " + this->search_algorithm_;
  if (this->time_limit_ > 0)
    command += " -T " + std::to_string(this->time_limit_);
  if (this->tolerance_ != 0.01f)
    command += " -t " + std::to_string(this->tolerance_);
  if (this->weight_ != 1.0f)
    command += " -w " + std::to_string(this->weight_);

  command += " " + domain_path + " " + problem_path;

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

  pclose(pipe);
  return output;
}

bool VhpopPlanner::has_solution(const std::string &plan_str) const {
  return plan_str.find("Time:") != std::string::npos &&
         plan_str.find("no plan") == std::string::npos;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(VhpopPlanner, omni_plan::Planner)