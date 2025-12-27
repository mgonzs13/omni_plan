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

SmtpPlanner::SmtpPlanner() : Planner() {
  // Add SMTP options as parameters
  this->add_ros_parameters({
      {"happenings_start", 1, happenings_start_},
      {"happenings_limit", -1, happenings_limit_},
      {"chain_length_limit", 2, chain_length_limit_},
      {"encoding", 0, encoding_},
      {"step_size", 1, step_size_},
  });
}

std::string SmtpPlanner::generate_plan(const std::string domain_path,
                                       const std::string problem_path) const {

  // Build command with options
  std::string command =
      ament_index_cpp::get_package_share_directory("easy_plan_smtp") +
      "/bin/SMTPlan";

  if (this->happenings_start_ != 1)
    command += " -l " + std::to_string(this->happenings_start_);
  if (this->happenings_limit_ != -1)
    command += " -u " + std::to_string(this->happenings_limit_);
  if (this->chain_length_limit_ != 2)
    command += " -c " + std::to_string(this->chain_length_limit_);
  if (this->encoding_ != 0)
    command += " -e " + std::to_string(this->encoding_);
  if (this->step_size_ != 1)
    command += " -s " + std::to_string(this->step_size_);

  command += " " + domain_path + " " + problem_path + " 2>&1";

  // Run SMTP planner
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