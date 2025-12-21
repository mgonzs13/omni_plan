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
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "easy_plan_smtp/smtp_planner.hpp"

using namespace easy_plan_smtp;

SmtpPlanner::SmtpPlanner() : Planner() {}

std::string SmtpPlanner::generate_plan(const std::string domain_path,
                                       const std::string problem_path) const {

  // Run SMTP planner (capture both stdout and stderr)
  std::string command =
      ament_index_cpp::get_package_share_directory("easy_plan_smtp") +
      "/planner/SMTPlan " + domain_path + " " + problem_path + " -u 1000 2>&1";
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

bool SmtpPlanner::has_solution(const std::string &plan_str) const {
  return plan_str.find("No plan found") == std::string::npos &&
         plan_str.find("Critical Errors") == std::string::npos;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(SmtpPlanner, easy_plan::Planner)