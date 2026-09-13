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

#include <sys/wait.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

#include "omni_plan/utils/package_share_path.hpp"

#include "omni_plan_lpg/lpg_planner.hpp"

using namespace omni_plan_lpg;

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

struct TempDirGuard {
  std::string path;
  explicit TempDirGuard(const std::string &p) : path(p) {}
  ~TempDirGuard() {
    if (!path.empty()) {
      std::error_code ec;
      std::filesystem::remove_all(path, ec);
    }
  }
  TempDirGuard(const TempDirGuard &) = delete;
  TempDirGuard &operator=(const TempDirGuard &) = delete;
};

} // namespace

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
                            {"advanced_time", false, this->advanced_time_},
                            {"timeout", 0, this->timeout_}});
}

std::string LpgPlanner::generate_plan(const std::string &domain_path,
                                      const std::string &problem_path) const {

  // Build command with options
  std::string command = shell_quote(
      omni_plan::utils::get_package_share_path("omni_plan_lpg") + "/bin/lpg");

  command += " -o " + shell_quote(domain_path);
  command += " -f " + shell_quote(problem_path);
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

  // Create a unique temporary directory for the plan output files. LPG
  // writes <prefix>_1.SOL, <prefix>_2.SOL, ... and the guard removes them all.
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::string dir_template = (temp_dir / "lpg_XXXXXX").string();
  std::vector<char> dir_buffer(dir_template.begin(), dir_template.end());
  dir_buffer.push_back('\0');
  char *dir = mkdtemp(dir_buffer.data());
  if (dir == nullptr) {
    return "";
  }
  TempDirGuard dir_guard(dir);
  std::string prefix = std::string(dir) + "/lpg";

  command += " -out " + shell_quote(prefix);
  command += " > /dev/null 2>&1";

  if (this->timeout_ > 0)
    command = "timeout -k 5 " + std::to_string(this->timeout_) + " " + command;

  int status = std::system(command.c_str()); // NOLINT
  if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "[lpg] LPG terminated abnormally" << std::endl;
    return "";
  }

  // LPG writes the plan to <prefix>_1.SOL
  std::string sol_path = prefix + "_1.SOL";
  std::ifstream ifs(sol_path);
  std::string output((std::istreambuf_iterator<char>(ifs)),
                     std::istreambuf_iterator<char>());

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

std::pair<std::string, std::vector<std::string>>
LpgPlanner::parse_action_line(std::string line) const {
  // LPG prints action names in uppercase; lowercase only the action token so
  // that parameter names keep their original case.
  size_t start = line.find('(');
  size_t end =
      start == std::string::npos ? std::string::npos : line.find(')', start);
  if (start != std::string::npos && end != std::string::npos &&
      end > start + 1) {
    size_t token_end = line.find_first_of(" \t", start + 1);
    if (token_end == std::string::npos || token_end > end) {
      token_end = end;
    }
    std::transform(line.begin() + start + 1, line.begin() + token_end,
                   line.begin() + start + 1,
                   [](unsigned char c) { return std::tolower(c); });
  }
  return Planner::parse_action_line(line);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LpgPlanner, omni_plan::Planner)
