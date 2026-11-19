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

#include <iostream>
#include <memory>

#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto graph = knowledge_graph::KnowledgeGraph::get_instance();

  std::cout << "Populating knowledge graph for parallel assembly demo..."
            << std::endl;

  // ----- Objects (nodes) -----
  // Types are inferred from the node's type field by KgPddlManager
  graph->create_node("robot1", "robot");
  graph->create_node("robot2", "robot");

  graph->create_node("workshop", "room");
  graph->create_node("storage_a", "room");
  graph->create_node("storage_b", "room");
  graph->create_node("assembly", "room");

  graph->create_node("comp_a", "component");
  graph->create_node("comp_b", "component");

  // ----- Initial facts (edges) -----
  // Robots start at workshop with full batteries
  // binary:  robot_at(robot, room)
  graph->create_edge("robot_at", "robot1", "workshop");
  graph->create_edge("robot_at", "robot2", "workshop");

  // unary:   battery_full(robot)  — self-edge encodes a unary predicate
  graph->create_edge("battery_full", "robot1", "robot1");
  graph->create_edge("battery_full", "robot2", "robot2");

  // Components in their respective storage rooms
  // binary:  component_at(component, room)
  graph->create_edge("component_at", "comp_a", "storage_a");
  graph->create_edge("component_at", "comp_b", "storage_b");

  // Room connections (bidirectional)
  // binary:  connected(room, room)
  graph->create_edge("connected", "workshop", "storage_a"); // branch 1 outbound
  graph->create_edge("connected", "storage_a", "workshop");
  graph->create_edge("connected", "workshop", "storage_b"); // branch 2 outbound
  graph->create_edge("connected", "storage_b", "workshop");
  graph->create_edge("connected", "storage_a", "assembly"); // branch 1 return
  graph->create_edge("connected", "assembly", "storage_a");
  graph->create_edge("connected", "storage_b", "assembly"); // branch 2 return
  graph->create_edge("connected", "assembly", "storage_b");
  graph->create_edge("is_assembly_room", "assembly", "assembly");

  // ----- Goal -----
  // assembled(comp_a, comp_b) — requires both branches to complete
  auto goal_edge = graph->create_edge("assembled", "comp_a", "comp_b");
  goal_edge.set_property<bool>("is_goal", true);
  graph->update_edge(goal_edge);

  std::cout << "Knowledge graph populated successfully!" << std::endl;
  std::cout << std::endl;

  // Sleep for a bit
  rclcpp::sleep_for(std::chrono::seconds(5));

  rclcpp::shutdown();
  return 0;
}
