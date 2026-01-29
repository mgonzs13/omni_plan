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
#include <cstdlib>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan_knowledge_base/knowledge_base_client.hpp"

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
    // Start knowledge base node
    system("ros2 run omni_plan_knowledge_base knowledge_base_node &");

    std::string cmd =
        "ros2 run yasmin_factory yasmin_factory_node --ros-args -r "
        "__node:=omni_plan_node -p "
        "state_machine_file:=" +
        ament_index_cpp::get_package_share_directory("omni_plan") +
        "/state_machines/planning_sm.xml" + " --params-file " +
        ament_index_cpp::get_package_share_directory("omni_plan_tests") +
        "/params/test.yaml &";
    system(cmd.c_str());
    kb_client_ =
        std::make_unique<omni_plan_knowledge_base::KnowledgeBaseClient>(
            "test_kb_client");

    // Wait a bit for the system to start
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  void TearDown() override {
    system("kill -9 $(pgrep -f yasmin_factory_node)");
    system("kill -9 $(pgrep -f knowledge_base_node)");

    // Cleanup the knowledge base
    kb_client_->clear();

    kb_client_.reset();
  }

  std::unique_ptr<omni_plan_knowledge_base::KnowledgeBaseClient> kb_client_;
};

TEST_F(FullSystemTest, FullIntegrationMove) {
  // Add types
  kb_client_->add_types({"robot", "room"});

  // Add objects
  kb_client_->add_object(omni_plan::pddl::Object("robot1", "robot"));
  kb_client_->add_object(omni_plan::pddl::Object("room1", "room"));
  kb_client_->add_object(omni_plan::pddl::Object("room2", "room"));
  kb_client_->add_object(omni_plan::pddl::Object("room3", "room"));

  // Add predicates
  kb_client_->add_predicate(
      omni_plan::pddl::Predicate("robot_at", {"robot", "room"}));
  kb_client_->add_predicate(
      omni_plan::pddl::Predicate("connected", {"room", "room"}));

  // Add facts
  kb_client_->add_fact(
      omni_plan::pddl::Predicate("robot_at", {"robot1", "room1"}));

  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room1", "room2"}));
  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room2", "room1"}));

  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room2", "room3"}));
  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room3", "room2"}));

  // Add goal
  kb_client_->add_goal(
      omni_plan::pddl::Predicate("robot_at", {"robot1", "room3"}));

  // Wait for the state machine to process
  while (kb_client_->has_goals()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Assert the robot is at room3
  auto facts = kb_client_->get_facts("robot_at");

  // Check robot_at(robot1, room3) is in facts
  bool found = false;
  for (const auto &fact : facts) {
    if (fact == omni_plan::pddl::Predicate("robot_at", {"robot1", "room3"})) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(FullSystemTest, FullIntegrationCharge) {
  // Add types
  kb_client_->add_types({"robot", "room"});

  // Add objects
  kb_client_->add_object(omni_plan::pddl::Object("robot1", "robot"));
  kb_client_->add_object(omni_plan::pddl::Object("room1", "room"));
  kb_client_->add_object(omni_plan::pddl::Object("room2", "room"));
  kb_client_->add_object(omni_plan::pddl::Object("room3", "room"));

  // Add predicates
  kb_client_->add_predicate(
      omni_plan::pddl::Predicate("robot_at", {"robot", "room"}));
  kb_client_->add_predicate(
      omni_plan::pddl::Predicate("connected", {"room", "room"}));
  kb_client_->add_predicate(
      omni_plan::pddl::Predicate("battery_low", {"robot"}));
  kb_client_->add_predicate(
      omni_plan::pddl::Predicate("battery_full", {"robot"}));
  kb_client_->add_predicate(
      omni_plan::pddl::Predicate("charging_point_at", {"room"}));

  // Add facts
  kb_client_->add_fact(
      omni_plan::pddl::Predicate("robot_at", {"robot1", "room1"}));
  kb_client_->add_fact(omni_plan::pddl::Predicate("battery_low", {"robot1"}));
  kb_client_->add_fact(
      omni_plan::pddl::Predicate("charging_point_at", {"room2"}));

  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room1", "room2"}));
  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room2", "room1"}));

  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room2", "room3"}));
  kb_client_->add_fact(
      omni_plan::pddl::Predicate("connected", {"room3", "room2"}));

  // Add goal
  kb_client_->add_goal(omni_plan::pddl::Predicate("battery_full", {"robot1"}));

  // Wait for the state machine to process
  while (kb_client_->has_goals()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Assert the robot has battery full
  EXPECT_TRUE(!kb_client_->get_facts("battery_full").empty());

  // Assert the robot does not have battery low
  EXPECT_FALSE(!kb_client_->get_facts("battery_low").empty());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
