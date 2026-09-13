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
#include <gtest/gtest.h>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_knowledge_base/kb_pddl_manager.hpp"
#include "omni_plan_knowledge_base/knowledge_base_client.hpp"
#include "omni_plan_knowledge_base/knowledge_base_node.hpp"

using namespace omni_plan_knowledge_base;
using namespace std::chrono_literals;

class KbPddlManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_shared<KnowledgeBaseNode>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    kb_client_ =
        std::make_shared<KnowledgeBaseClient>("kb_manager_test_client");

    std::this_thread::sleep_for(100ms);
  }

  void TearDown() override {
    kb_manager_.reset();
    kb_client_.reset();
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  void setup_world() {
    kb_client_->add_type("robot");
    kb_client_->add_type("location");
    kb_client_->add_object("robot1", "robot");
    kb_client_->add_object("loc1", "location");
    kb_client_->add_object("loc2", "location");
    kb_client_->add_predicate("at", {"robot", "location"});
  }

  std::shared_ptr<KnowledgeBaseNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::shared_ptr<KnowledgeBaseClient> kb_client_;
  std::shared_ptr<KbPddlManager> kb_manager_;
};

TEST_F(KbPddlManagerTest, HasGoalsReturnsFalseQuicklyWhenNoGoals) {
  kb_manager_ = std::make_shared<KbPddlManager>();

  auto start = std::chrono::steady_clock::now();
  bool result = kb_manager_->has_goals();
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result);
  EXPECT_LT(elapsed, 2s);
}

TEST_F(KbPddlManagerTest, HasGoalsReturnsTrueAfterGoalAdded) {
  kb_manager_ = std::make_shared<KbPddlManager>();
  setup_world();

  EXPECT_FALSE(kb_manager_->has_goals());
  EXPECT_TRUE(kb_client_->add_goal("at", {"robot1", "loc2"}));

  auto start = std::chrono::steady_clock::now();
  bool result = kb_manager_->has_goals();
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_TRUE(result);
  EXPECT_LT(elapsed, 2s);
}

TEST_F(KbPddlManagerTest, ClearGoalsRemovesGoals) {
  kb_manager_ = std::make_shared<KbPddlManager>();
  setup_world();

  EXPECT_TRUE(kb_client_->add_goal("at", {"robot1", "loc2"}));
  EXPECT_TRUE(kb_manager_->has_goals());

  EXPECT_TRUE(kb_manager_->clear_goals());
  EXPECT_FALSE(kb_manager_->has_goals());
}

TEST_F(KbPddlManagerTest, DestroyedManagerSurvivesLateUpdates) {
  {
    KbPddlManager manager;
    setup_world();
    EXPECT_TRUE(kb_client_->add_goal("at", {"robot1", "loc2"}));
  }

  EXPECT_TRUE(kb_client_->add_type("extra_type"));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
