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

#include <string>

#include "easy_plan_popf/popf_planner.hpp"

using namespace easy_plan_popf;

PopfPlanner::PopfPlanner() : Planner() {}

std::string PopfPlanner::generate_plan(const std::string domain_path,
                                       const std::string problem_path) const {

  // Run POPF planner
  std::string command =
      "ros2 run popf popf " + domain_path + " " + problem_path;
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

bool PopfPlanner::has_solution(const std::string &plan_output) const {
  return plan_output.find("Solution Found") != std::string::npos;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PopfPlanner, easy_plan::Planner)