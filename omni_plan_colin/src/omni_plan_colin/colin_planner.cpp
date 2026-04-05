// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "omni_plan_colin/colin_planner.hpp"

using namespace omni_plan_colin;

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
       {"total_order", false, this->total_order_}});
}

std::string ColinPlanner::generate_plan(const std::string domain_path,
                                        const std::string problem_path) const {

  // Build command with options
  std::string command =
      ament_index_cpp::get_package_share_directory("omni_plan_colin") +
      "/bin/colin";

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

  command += " " + domain_path + " " + problem_path;

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

  pclose(pipe);
  return output;
}

bool ColinPlanner::has_solution(const std::string &plan_output) const {
  return plan_output.find("Solution Found") != std::string::npos;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ColinPlanner, omni_plan::Planner)
