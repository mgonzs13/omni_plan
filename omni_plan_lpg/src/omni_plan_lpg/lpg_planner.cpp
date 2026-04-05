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

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "omni_plan_lpg/lpg_planner.hpp"

using namespace omni_plan_lpg;

LpgPlanner::LpgPlanner() : Planner() {
  // Add LPG options as parameters
  this->add_ros_parameters({{"num_solutions", 1, this->num_solutions_},
                            {"heuristic", 1, this->heuristic_},
                            {"restarts", 50, this->restarts_},
                            {"search_steps", 500, this->search_steps_},
                            {"noise", 0.1, this->noise_},
                            {"nobestfirst", false, this->nobestfirst_},
                            {"onlybestfirst", false, this->onlybestfirst_},
                            {"seed", 0, this->seed_},
                            {"i_choice", 2, this->i_choice_},
                            {"cputime", 0, this->cputime_},
                            {"advanced_time", false, this->advanced_time_}});
}

std::string LpgPlanner::generate_plan(const std::string domain_path,
                                      const std::string problem_path) const {

  // Build command with options
  std::string command =
      ament_index_cpp::get_package_share_directory("omni_plan_lpg") +
      "/bin/lpg";

  command += " -o " + domain_path;
  command += " -f " + problem_path;
  command += " -n " + std::to_string(this->num_solutions_);

  if (this->heuristic_ != 1)
    command += " -h " + std::to_string(this->heuristic_);
  if (this->restarts_ != 50)
    command += " -restarts " + std::to_string(this->restarts_);
  if (this->search_steps_ != 500)
    command += " -search_steps " + std::to_string(this->search_steps_);
  if (this->noise_ != 0.1)
    command += " -noise " + std::to_string(this->noise_);
  if (this->nobestfirst_)
    command += " -nobestfirst";
  if (this->onlybestfirst_)
    command += " -onlybestfirst";
  if (this->seed_ != 0)
    command += " -seed " + std::to_string(this->seed_);
  if (this->i_choice_ != 2)
    command += " -i_choice " + std::to_string(this->i_choice_);
  if (this->cputime_ > 0)
    command += " -cputime " + std::to_string(this->cputime_);
  if (this->advanced_time_)
    command += " -AdvancedTime";

  // Generate a unique prefix under /tmp for the plan output file.
  // mkstemp creates the file; we immediately remove it so LPG can write
  // <prefix>_1.SOL at that path.
  char tmp_prefix[] = "/tmp/lpg_XXXXXX";
  int fd = mkstemp(tmp_prefix);
  if (fd == -1) {
    return "";
  }
  close(fd);
  std::remove(tmp_prefix);

  command += " -out " + std::string(tmp_prefix);
  command += " > /dev/null 2>&1";

  std::system(command.c_str()); // NOLINT

  // LPG writes the plan to <prefix>_1.SOL (for -n 1)
  std::string sol_path = std::string(tmp_prefix) + "_1.SOL";
  std::ifstream ifs(sol_path);
  std::string output((std::istreambuf_iterator<char>(ifs)),
                     std::istreambuf_iterator<char>());

  std::remove(sol_path.c_str());

  return output;
}

bool LpgPlanner::has_solution(const std::string &plan_output) const {
  // The SOL file is only written when a solution exists; an empty string
  // therefore means no solution was found.
  return !plan_output.empty();
}

std::vector<std::string>
LpgPlanner::get_lines_with_actions(const std::string &plan_str) const {
  std::vector<std::string> action_lines;
  std::stringstream ss(plan_str);
  std::string line;

  while (std::getline(ss, line)) {
    // Must contain the four bracket characters that delimit an action
    if (line.find('(') == std::string::npos ||
        line.find(')') == std::string::npos ||
        line.find('[') == std::string::npos ||
        line.find(']') == std::string::npos) {
      continue;
    }

    // The part before the first ':' must be a valid floating-point timestamp;
    // this filters out header lines such as "   Time: (ACTION) [...]"
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

float LpgPlanner::parse_duration(const std::string &line) const {
  size_t bracket_start = line.find('[');
  size_t bracket_end = line.find(']', bracket_start);
  if (bracket_start == std::string::npos || bracket_end == std::string::npos) {
    return 0.0f;
  }

  // SOL file format: "(ACTION PARAMS)[10.000] ;; cost 1.000"
  std::string bracket_content =
      line.substr(bracket_start + 1, bracket_end - bracket_start - 1);

  try {
    return std::stof(bracket_content);
  } catch (...) {
    return 0.0f;
  }
}

std::pair<std::string, std::vector<std::string>>
LpgPlanner::parse_action_line(std::string line) const {
  // LPG outputs action names and parameters in uppercase; convert to lowercase
  // so that they match the lowercase keys stored in the actions map.
  std::transform(line.begin(), line.end(), line.begin(), ::tolower);
  return Planner::parse_action_line(line);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LpgPlanner, omni_plan::Planner)
