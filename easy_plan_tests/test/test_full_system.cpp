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

#include <cstdlib>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "knowledge_graph/knowledge_graph.hpp"

/**
 * @brief Test fixture for plugin loading tests
 */
class FullSystemTest : public ::testing::Test {
protected:
  static void SetUpTestCase() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    std::string cmd =
        "ros2 run yasmin_factory yasmin_factory_node --ros-args -p "
        "state_machine_file:=" +
        ament_index_cpp::get_package_share_directory("easy_plan") +
        "/state_machines/planning_sm.xml" + " --params-file " +
        ament_index_cpp::get_package_share_directory("easy_plan_tests") +
        "/params/test.yaml &";
    system(cmd.c_str());
    kg_ = knowledge_graph::KnowledgeGraph::get_instance();
  }

  void TearDown() override {
    system("kill -9 $(pgrep -f yasmin_factory_node)");

    // Cleanup the knowledge graph
    for (const auto &node : kg_->get_nodes()) {
      kg_->remove_node(node);
    }

    kg_.reset();
  }

  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
};

TEST_F(FullSystemTest, FullIntegrationMove) {
  auto robot = kg_->create_node("robot1", "robot");
  auto room1 = kg_->create_node("room1", "room");
  auto room2 = kg_->create_node("room2", "room");
  auto room3 = kg_->create_node("room3", "room");

  kg_->create_edge("robot_at", robot.get_name(), room1.get_name());

  kg_->create_edge("connected", room1.get_name(), room2.get_name());
  kg_->create_edge("connected", room2.get_name(), room1.get_name());

  kg_->create_edge("connected", room2.get_name(), room3.get_name());
  kg_->create_edge("connected", room3.get_name(), room2.get_name());

  auto at_edge =
      kg_->create_edge("robot_at", robot.get_name(), room3.get_name());
  at_edge.set_property<bool>("is_goal", true);
  kg_->update_edge(at_edge);

  // Wait for the state machine to process using while
  at_edge = kg_->get_edge("robot_at", robot.get_name(), room3.get_name());

  while (at_edge.has_property("is_goal") &&
         at_edge.get_property<bool>("is_goal")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    at_edge = kg_->get_edge("robot_at", robot.get_name(), room3.get_name());
  }

  // Assert the number of edges
  EXPECT_EQ(kg_->get_num_edges(), 5);

  // Assert the robot is at room3
  at_edge = kg_->get_edge("robot_at", robot.get_name(), room3.get_name());
  EXPECT_FALSE(at_edge.has_property("is_goal"));
}

TEST_F(FullSystemTest, FullIntegrationCharge) {
  auto robot = kg_->create_node("robot1", "robot");
  auto room1 = kg_->create_node("room1", "room");
  auto room2 = kg_->create_node("room2", "room");
  auto room3 = kg_->create_node("room3", "room");

  kg_->create_edge("robot_at", robot.get_name(), room1.get_name());
  kg_->create_edge("battery_low", robot.get_name(), robot.get_name());
  kg_->create_edge("charging_point_at", room2.get_name(), room2.get_name());

  kg_->create_edge("connected", room1.get_name(), room2.get_name());
  kg_->create_edge("connected", room2.get_name(), room1.get_name());

  kg_->create_edge("connected", room2.get_name(), room3.get_name());
  kg_->create_edge("connected", room3.get_name(), room2.get_name());

  auto edge =
      kg_->create_edge("battery_full", robot.get_name(), robot.get_name());
  edge.set_property<bool>("is_goal", true);
  kg_->update_edge(edge);

  // Wait for the state machine to process using while
  edge = kg_->get_edge("battery_full", robot.get_name(), robot.get_name());

  while (edge.has_property("is_goal") && edge.get_property<bool>("is_goal")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    edge = kg_->get_edge("battery_full", robot.get_name(), robot.get_name());
  }

  // Assert the number of edges
  EXPECT_EQ(kg_->get_num_edges(), 7);

  // Assert the robot has battery full
  edge = kg_->get_edge("battery_full", robot.get_name(), robot.get_name());
  EXPECT_FALSE(edge.has_property("is_goal"));

  // Assert the robot does not have battery low
  EXPECT_FALSE(
      kg_->has_edge("battery_low", robot.get_name(), robot.get_name()));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
