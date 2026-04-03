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

#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_tui/tui_node.hpp"

std::shared_ptr<omni_plan_tui::TuiNode> g_node = nullptr;

void signal_handler(int signum) {
  (void)signum;
  if (g_node) {
    g_node->stop();
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  g_node = std::make_shared<omni_plan_tui::TuiNode>();
  g_node->run();

  rclcpp::spin(g_node);

  g_node->stop();
  g_node.reset();

  rclcpp::shutdown();
  return 0;
}
