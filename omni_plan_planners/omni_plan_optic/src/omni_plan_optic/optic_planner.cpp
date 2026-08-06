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

#include <sstream>
#include <string>
#include <vector>

#include "omni_plan/utils/package_share_path.hpp"

#include "omni_plan_optic/optic_planner.hpp"

using namespace omni_plan_optic;

OpticPlanner::OpticPlanner() : Planner() {
  // Add OPTIC options as parameters
  this->add_ros_parameters(
      {{"no_optimise", false, this->no_optimise_},
       {"abstract_timed_literals", false, this->abstract_timed_literals_},
       {"cost_limit", -1.0, this->cost_limit_},
       {"disable_best_first", false, this->disable_best_first_},
       {"skip_ehc", false, this->skip_ehc_},
       {"standard_ehc", false, this->standard_ehc_},
       {"disable_helpful_pruning", false, this->disable_helpful_pruning_},
       {"disable_compression_safe", false, this->disable_compression_safe_},
       {"tie_breaking_rpg", false, this->tie_breaking_rpg_},
       {"sort_initial_layer", false, this->sort_initial_layer_},
       {"disable_tie_breaking_search", false,
        this->disable_tie_breaking_search_},
       {"full_ff_helpful", false, this->full_ff_helpful_},
       {"total_order", false, this->total_order_}});
}

std::string OpticPlanner::generate_plan(const std::string &domain_path,
                                        const std::string &problem_path) const {

  // Build command with options
  std::string command =
      omni_plan::utils::get_package_share_path("omni_plan_optic") +
      "/bin/optic-clp";

  if (this->no_optimise_)
    command += " -N";
  if (this->abstract_timed_literals_)
    command += " -0";
  if (this->cost_limit_ >= 0.0)
    command += " -n" + std::to_string(this->cost_limit_);
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
  if (this->tie_breaking_rpg_)
    command += " -c";
  if (this->sort_initial_layer_)
    command += " -S";
  if (this->disable_tie_breaking_search_)
    command += " -m";
  if (this->full_ff_helpful_)
    command += " -F";
  if (this->total_order_)
    command += " -T";

  command += " " + domain_path + " " + problem_path;

  // Run OPTIC planner
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

bool OpticPlanner::has_solution(const std::string &plan_output) const {
  return plan_output.find("Solution Found") != std::string::npos;
}

std::vector<std::string>
OpticPlanner::get_lines_with_actions(const std::string &plan_str) const {
  std::vector<std::string> action_lines;

  // OPTIC may print the plan twice; take only the block after the last
  // ";;;; Solution Found" marker to avoid duplicates.
  size_t solution_pos = plan_str.rfind(";;;; Solution Found");
  if (solution_pos == std::string::npos) {
    return action_lines;
  }

  std::stringstream ss(plan_str.substr(solution_pos));
  std::string line;
  while (std::getline(ss, line)) {
    if (line.find('(') != std::string::npos &&
        line.find(')') != std::string::npos &&
        line.find('[') != std::string::npos &&
        line.find(']') != std::string::npos) {
      action_lines.push_back(line);
    }
  }

  return action_lines;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(OpticPlanner, omni_plan::Planner)
