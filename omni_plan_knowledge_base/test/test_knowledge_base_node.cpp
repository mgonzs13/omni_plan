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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_knowledge_base/knowledge_base_client.hpp"
#include "omni_plan_knowledge_base/knowledge_base_node.hpp"

using namespace omni_plan_knowledge_base;
using namespace std::chrono_literals;

class KnowledgeBaseNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<KnowledgeBaseNode>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Start executor in a separate thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Create KnowledgeBaseClient
    kb_client_ = std::make_shared<KnowledgeBaseClient>("test_kb_client");

    // Give services time to start up
    std::this_thread::sleep_for(100ms);
  }

  void TearDown() override {
    kb_client_.reset();
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  std::shared_ptr<KnowledgeBaseNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::shared_ptr<KnowledgeBaseClient> kb_client_;
};

// ==================== Type Service Tests ====================
TEST_F(KnowledgeBaseNodeTest, AddTypeService) {
  bool success = kb_client_->add_type("robot");
  EXPECT_TRUE(success);
}

TEST_F(KnowledgeBaseNodeTest, GetTypesService) {
  // First add a type
  bool success = kb_client_->add_type("robot");
  EXPECT_TRUE(success);

  // Then get types
  auto types = kb_client_->get_types();
  EXPECT_EQ(types.size(), 1u);
  EXPECT_EQ(types[0], "robot");
}

TEST_F(KnowledgeBaseNodeTest, RemoveTypeService) {
  // First add a type
  bool success = kb_client_->add_type("robot");
  EXPECT_TRUE(success);

  // Then remove it
  bool removed = kb_client_->remove_type("robot");
  EXPECT_TRUE(removed);
}

// ==================== Object Service Tests ====================
TEST_F(KnowledgeBaseNodeTest, AddObjectService) {
  // First add the type
  kb_client_->add_type("robot");

  // Now add the object
  bool success = kb_client_->add_object("robot1", "robot");
  EXPECT_TRUE(success);
}

TEST_F(KnowledgeBaseNodeTest, GetObjectsService) {
  // First add the type
  kb_client_->add_type("robot");

  // Then add an object
  kb_client_->add_object("robot1", "robot");

  // Then get objects
  auto objects = kb_client_->get_objects();
  EXPECT_EQ(objects.size(), 1u);
  EXPECT_EQ(objects[0].get_name(), "robot1");
  EXPECT_EQ(objects[0].get_type(), "robot");
}

// ==================== Fact Service Tests ====================
TEST_F(KnowledgeBaseNodeTest, AddFactService) {
  // First add types
  kb_client_->add_type("robot");
  kb_client_->add_type("location");

  // Add objects
  kb_client_->add_object("robot1", "robot");
  kb_client_->add_object("loc1", "location");

  // Add predicate definition
  kb_client_->add_predicate("at", {"robot", "location"});

  // Now add fact
  bool success = kb_client_->add_fact("at", {"robot1", "loc1"});
  EXPECT_TRUE(success);
}

TEST_F(KnowledgeBaseNodeTest, GetFactsService) {
  // First add types
  kb_client_->add_type("robot");
  kb_client_->add_type("location");

  // Add objects
  kb_client_->add_object("robot1", "robot");
  kb_client_->add_object("loc1", "location");

  // Add predicate definition
  kb_client_->add_predicate("at", {"robot", "location"});

  // Then add a fact
  kb_client_->add_fact("at", {"robot1", "loc1"});

  // Then get facts
  auto facts = kb_client_->get_facts();
  EXPECT_EQ(facts.size(), 1u);
  EXPECT_EQ(facts[0].get_name(), "at");
  EXPECT_EQ(facts[0].get_args().size(), 2u);
}

// ==================== Goal Service Tests ====================
TEST_F(KnowledgeBaseNodeTest, AddGoalService) {
  // First add types
  kb_client_->add_type("robot");
  kb_client_->add_type("location");

  // Add objects
  kb_client_->add_object("robot1", "robot");
  kb_client_->add_object("loc2", "location");

  // Add predicate definition
  kb_client_->add_predicate("at", {"robot", "location"});

  // Now add goal
  bool success = kb_client_->add_goal("at", {"robot1", "loc2"});
  EXPECT_TRUE(success);
}

TEST_F(KnowledgeBaseNodeTest, GetGoalsService) {
  // First add types
  kb_client_->add_type("robot");
  kb_client_->add_type("location");

  // Add objects
  kb_client_->add_object("robot1", "robot");
  kb_client_->add_object("loc2", "location");

  // Add predicate definition
  kb_client_->add_predicate("at", {"robot", "location"});

  // Then add a goal
  kb_client_->add_goal("at", {"robot1", "loc2"});

  // Then get goals
  auto goals = kb_client_->get_goals();
  EXPECT_EQ(goals.size(), 1u);
  EXPECT_EQ(goals[0].get_name(), "at");
  EXPECT_EQ(goals[0].get_args().size(), 2u);
}

// ==================== Clear Service Tests ====================
TEST_F(KnowledgeBaseNodeTest, ClearAllService) {
  // Add some data
  kb_client_->add_type("robot");
  kb_client_->add_type("location");
  kb_client_->add_object("robot1", "robot");
  kb_client_->add_predicate("at", {"robot", "location"});
  kb_client_->add_fact("at", {"robot1", "loc1"});

  // Note: KnowledgeBaseClient doesn't have a clear method yet
  // This test would need to be implemented when that's added
  // For now, we'll skip testing the clear functionality

  // Verify data exists
  auto types = kb_client_->get_types();
  EXPECT_FALSE(types.empty());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
