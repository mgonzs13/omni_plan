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

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

#include "omni_plan/utils/package_share_path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan_knowledge_base/knowledge_base_client.hpp"

namespace {

/**
 * @brief Start a command in its own session/process group and return its PID.
 *
 * setsid puts the command (and everything it spawns) in a dedicated process
 * group, so the whole tree can be killed with kill(-pid, SIGKILL).  Output is
 * discarded to avoid filling the popen pipe and blocking the node.
 */
pid_t spawn_detached(const std::string &command) {
  const std::string full = "setsid " + command + " >/dev/null 2>&1 & echo $!";
  FILE *pipe = popen(full.c_str(), "r");
  if (pipe == nullptr) {
    return -1;
  }

  char buffer[64] = {0};
  const bool read_ok = std::fgets(buffer, sizeof(buffer), pipe) != nullptr;
  pclose(pipe);
  if (!read_ok) {
    return -1;
  }
  return static_cast<pid_t>(std::atoi(buffer));
}

/**
 * @brief Kill a process group started with spawn_detached() and wait for it.
 */
void kill_process_group(pid_t pid) {
  if (pid <= 0) {
    return;
  }
  ::kill(-pid, SIGKILL);

  for (int i = 0; i < 100; ++i) {
    if (::kill(-pid, 0) != 0 && errno == ESRCH) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

} // namespace

/**
 * @brief Test fixture for plugin loading tests
 */
class FullSystemTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Start knowledge base node
    kb_pid_ =
        spawn_detached("ros2 run omni_plan_knowledge_base knowledge_base_node");

    std::string cmd =
        "ros2 run yasmin_factory yasmin_factory_node --ros-args -r "
        "__node:=omni_plan_node -p state_machine_file:=\"" +
        omni_plan::utils::get_package_share_path("omni_plan") +
        "/state_machines/planning_sm.xml\" --params-file \"" +
        omni_plan::utils::get_package_share_path("omni_plan_tests") +
        "/params/test.yaml\"";
    factory_pid_ = spawn_detached(cmd);

    kb_client_ =
        std::make_unique<omni_plan_knowledge_base::KnowledgeBaseClient>(
            "test_kb_client");

    // Wait a bit for the system to start
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  void TearDown() override {
    // Never call services after the nodes have been killed; killing the exact
    // PIDs captured at spawn avoids matching (and killing) the shell itself.
    kill_process_group(factory_pid_);
    kill_process_group(kb_pid_);
    factory_pid_ = -1;
    kb_pid_ = -1;

    kb_client_.reset();
  }

  std::unique_ptr<omni_plan_knowledge_base::KnowledgeBaseClient> kb_client_;
  pid_t factory_pid_ = -1;
  pid_t kb_pid_ = -1;
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
  auto start = std::chrono::steady_clock::now();
  while (kb_client_->has_goals()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(30))
      break;
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
  auto start = std::chrono::steady_clock::now();
  while (kb_client_->has_goals()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(30))
      break;
  }

  // Assert the robot has battery full
  EXPECT_TRUE(!kb_client_->get_facts("battery_full").empty());

  // Assert the robot does not have battery low
  EXPECT_FALSE(!kb_client_->get_facts("battery_low").empty());
}

int main(int argc, char **argv) {
  // Isolate this test from any other ROS 2 stack running on the machine (and
  // from parallel test runs).  Must happen before rclcpp::init() so that both
  // this process and the spawned nodes use the same private domain.
  char domain_id[16];
  std::snprintf(domain_id, sizeof(domain_id), "%d", 100 + (getpid() % 100));
  setenv("ROS_DOMAIN_ID", domain_id, 1);

  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
