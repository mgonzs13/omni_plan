#include <chrono>
#include <iostream>
#include <memory>

#include "knowledge_graph/knowledge_graph.hpp"
#include "rclcpp/rclcpp.hpp"

void create_rooms(knowledge_graph::KnowledgeGraph *graph) {
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
}

void set_goal(knowledge_graph::KnowledgeGraph *graph, const std::string &room) {
  auto robot_at_edges = graph->get_edges_by_type("robot_at");
  for (const auto &edge : robot_at_edges) {
    bool is_goal = false;
    try {
      is_goal = edge.get_property<bool>("is_goal");
    } catch (...) {
    }
    if (is_goal) {
      graph->remove_edge(edge);
    }
  }

  auto goal_edge = graph->create_edge("robot_at", "leia", room);
  goal_edge.set_property<bool>("is_goal", true);
  graph->update_edge(goal_edge);
}

void print_graph(knowledge_graph::KnowledgeGraph *graph) {
  auto edges = graph->get_edges();
  std::cout << "  Edges:" << std::endl;
  for (const auto &edge : edges) {
    bool is_goal = false;
    try {
      is_goal = edge.get_property<bool>("is_goal");
    } catch (...) {
    }
    std::cout << "    " << edge.to_string() << (is_goal ? " [GOAL]" : "")
              << std::endl;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto graph = knowledge_graph::KnowledgeGraph::get_instance();

  std::cout << "=== Cache Planner KG Demo ===" << std::endl;
  std::cout << std::endl;

  std::cout << "Setting up domain..." << std::endl;
  create_rooms(graph.get());
  std::cout << std::endl;

  // Goal 1: initial call — cache miss
  std::cout << "--- Goal 1: robot_at(leia, bathroom) [cache miss] ---"
            << std::endl;
  set_goal(graph.get(), "bathroom");
  print_graph(graph.get());
  std::cout << "  Waiting for planner to generate plan..." << std::endl;
  rclcpp::sleep_for(std::chrono::seconds(20));
  std::cout << std::endl;

  // Goal 2: exact same problem — exact cache hit
  std::cout << "--- Goal 2: robot_at(leia, kitchen) [exact hit] ---"
            << std::endl;
  set_goal(graph.get(), "kitchen");
  print_graph(graph.get());
  std::cout << "  Planner returns cached plan immediately." << std::endl;
  rclcpp::sleep_for(std::chrono::seconds(20));
  std::cout << std::endl;

  // Goal 3: different target room, same topology — structural cache hit
  std::cout << "--- Goal 3: robot_at(leia, bathroom) [structural hit] ---"
            << std::endl;
  set_goal(graph.get(), "bathroom");
  print_graph(graph.get());
  std::cout << "  Planner adapts cached plan for new target." << std::endl;
  rclcpp::sleep_for(std::chrono::seconds(20));
  std::cout << std::endl;

  // Goal 4: another different target room — structural cache hit
  std::cout << "--- Goal 4: robot_at(leia, chargingroom) [structural hit] ---"
            << std::endl;
  set_goal(graph.get(), "chargingroom");
  print_graph(graph.get());
  std::cout << "  Planner adapts cached plan for new target." << std::endl;
  rclcpp::sleep_for(std::chrono::seconds(20));
  std::cout << std::endl;

  std::cout << "Demo complete." << std::endl;

  rclcpp::shutdown();
  return 0;
}
