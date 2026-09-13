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
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "omni_plan/utils/package_share_path.hpp"
#include "omni_plan/utils/temp_file_guard.hpp"

#include "omni_plan_smtp/smtp_planner.hpp"

using namespace omni_plan_smtp;

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

SmtpPlanner::SmtpPlanner() : Planner() {
  // Add SMTP options as parameters
  this->add_ros_parameters({
      {"happenings_start", 1, happenings_start_},
      {"happenings_limit", -1, happenings_limit_},
      {"chain_length_limit", 2, chain_length_limit_},
      {"encoding", 0, encoding_},
      {"step_size", 1, step_size_},
      {"timeout", 0, timeout_},
  });
}

std::string SmtpPlanner::generate_plan(const std::string &domain_path,
                                       const std::string &problem_path) const {

  // Capture diagnostics separately so that only plan output is parsed.
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::string err_template = (temp_dir / "smtp_stderr_XXXXXX").string();
  std::vector<char> err_buffer(err_template.begin(), err_template.end());
  err_buffer.push_back('\0');
  int err_fd = mkstemp(err_buffer.data());
  if (err_fd == -1) {
    std::cerr << "[smtp] Failed to create temporary stderr file" << std::endl;
    return "";
  }
  close(err_fd);
  std::string err_path(err_buffer.data());
  omni_plan::utils::TempFileGuard err_guard(err_path.c_str());

  // Build command with options
  std::string command =
      shell_quote(omni_plan::utils::get_package_share_path("omni_plan_smtp") +
                  "/bin/SMTPlan");

  command += " " + shell_quote(domain_path) + " " + shell_quote(problem_path);

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

  command += " 2> " + shell_quote(err_path);

  if (this->timeout_ > 0)
    command = "timeout -k 5 " + std::to_string(this->timeout_) + " " + command;

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

  int status = pclose(pipe);
  if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::ifstream err_stream(err_path);
    std::string err_output((std::istreambuf_iterator<char>(err_stream)),
                           std::istreambuf_iterator<char>());
    std::cerr << "[smtp] SMTPlan terminated abnormally";
    if (!err_output.empty()) {
      std::cerr << ": " << err_output;
    } else {
      std::cerr << std::endl;
    }
    return "";
  }

  return output;
}

bool SmtpPlanner::has_solution(const std::string &plan_str) const {
  return !plan_str.empty() && plan_str.find("(") != std::string::npos &&
         plan_str.find(")") != std::string::npos &&
         plan_str.find("[") != std::string::npos &&
         plan_str.find("]") != std::string::npos &&
         plan_str.find("No plan found") == std::string::npos &&
         plan_str.find("Critical Errors") == std::string::npos;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(SmtpPlanner, omni_plan::Planner)