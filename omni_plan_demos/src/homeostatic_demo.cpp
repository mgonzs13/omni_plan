#include <chrono>
#include <iostream>
#include <memory>

#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto graph = knowledge_graph::KnowledgeGraph::get_instance();

  std::cout << "Populating knowledge graph for homeostatic planner demo..."
            << std::endl;

  graph->create_node("leia", "robot");

  graph->create_node("entrance", "room");
  graph->create_node("kitchen", "room");
  graph->create_node("bedroom", "room");
  graph->create_node("dinning", "room");
  graph->create_node("bathroom", "room");
  graph->create_node("chargingroom", "room");

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

  graph->create_edge("charging_point_at", "chargingroom", "chargingroom");
  graph->create_edge("battery_full", "leia", "leia");
  graph->create_edge("robot_at", "leia", "entrance");

  auto goal_edge = graph->create_edge("robot_at", "leia", "bathroom");
  goal_edge.set_property<bool>("is_goal", true);
  graph->update_edge(goal_edge);

  std::cout << "Knowledge graph populated successfully!" << std::endl;
  std::cout << std::endl;

  auto nodes = graph->get_nodes();
  std::cout << "Nodes (" << nodes.size() << "):" << std::endl;
  for (const auto &node : nodes) {
    std::cout << "  " << node.to_string() << std::endl;
  }

  auto edges = graph->get_edges();
  std::cout << "Edges (" << edges.size() << "):" << std::endl;
  for (const auto &edge : edges) {
    bool is_goal = false;
    try {
      is_goal = edge.get_property<bool>("is_goal");
    } catch (...) {
    }
    std::cout << "  " << edge.to_string() << (is_goal ? " [GOAL]" : "")
              << std::endl;
  }

  rclcpp::sleep_for(std::chrono::seconds(10));

  rclcpp::shutdown();
  return 0;
}
