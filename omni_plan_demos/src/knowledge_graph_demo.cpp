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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the knowledge graph demo node
  auto graph = knowledge_graph::KnowledgeGraph::get_instance();

  // Create robot instance
  graph->create_node("leia", "robot");

  // Create room instances
  graph->create_node("entrance", "room");

  graph->create_node("kitchen", "room");

  graph->create_node("bedroom", "room");

  graph->create_node("dinning", "room");

  graph->create_node("bathroom", "room");

  graph->create_node("chargingroom", "room");

  // Connected predicates (bidirectional)
  graph->create_edge("connected", "entrance", "dinning");
  graph->create_edge("connected", "dinning", "entrance");

  graph->create_edge("connected", "dinning", "kitchen");
  graph->create_edge("connected", "kitchen", "dinning");

  graph->create_edge("connected", "dinning", "bedroom");
  graph->create_edge("connected", "bedroom", "dinning");

  graph->create_edge("connected", "bathroom", "bedroom");
  graph->create_edge("connected", "bedroom", "bathroom");

  graph->create_edge("connected", "chargingroom", "kitchen");
  graph->create_edge("connected", "kitchen", "chargingroom");

  // Other predicates
  graph->create_edge("charging_point_at", "chargingroom", "chargingroom");
  graph->create_edge("battery_low", "leia", "leia");

  graph->create_edge("robot_at", "leia", "entrance");

  // Goal predicate
  auto goal_edge = graph->create_edge("robot_at", "leia", "bathroom");
  goal_edge.set_property<bool>("is_goal", true);
  graph->update_edge(goal_edge);

  std::cout << "Knowledge Graph Demo" << std::endl;
  std::cout << "====================" << std::endl;

  auto nodes = graph->get_nodes();
  std::cout << "Nodes (" << nodes.size() << "):" << std::endl;
  for (const auto &node : nodes) {
    std::cout << "  " << node.to_string() << std::endl;
  }

  auto edges = graph->get_edges();
  std::cout << "Edges (" << edges.size() << "):" << std::endl;
  for (const auto &edge : edges) {
    std::cout << "  " << edge.to_string() << std::endl;
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}