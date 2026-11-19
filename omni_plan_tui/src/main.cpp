// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
