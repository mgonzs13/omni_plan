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
#include <vector>

#include "omni_plan_yasmin/yasmin_action.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace omni_plan_yasmin;

/**
 * @brief Concrete implementation of YasminAction for testing
 */
class TestYasminAction : public YasminAction {
public:
  TestYasminAction(
      const std::string &name,
      const std::vector<std::pair<std::string, std::string>> &params = {},
      bool enable_viewer_pub = false)
      : YasminAction(name, params) {

    this->enable_viewer_pub_ = enable_viewer_pub;

    // Create a simple state machine that just succeeds
    auto success_state = std::make_shared<yasmin::CbState>(
        yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED},
        [](yasmin::Blackboard::SharedPtr) -> std::string {
          return yasmin_ros::basic_outcomes::SUCCEED;
        });

    this->add_state("SUCCESS_STATE", success_state,
                    {{yasmin_ros::basic_outcomes::SUCCEED,
                      yasmin_ros::basic_outcomes::SUCCEED}});
  }
};

/**
 * @brief Test fixture for YasminAction tests
 */
class YasminActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    simple_action_ = std::make_shared<TestYasminAction>("simple_yasmin");
    move_action_ = std::make_shared<TestYasminAction>(
        "move_yasmin",
        std::vector<std::pair<std::string, std::string>>{
            {"robot", "robot"}, {"from", "location"}, {"to", "location"}});
  }

  std::shared_ptr<TestYasminAction> simple_action_;
  std::shared_ptr<TestYasminAction> move_action_;
};

TEST_F(YasminActionTest, ConstructAction) {
  EXPECT_NE(simple_action_, nullptr);
  EXPECT_NE(move_action_, nullptr);
}

TEST_F(YasminActionTest, RunAction) {
  // Run the action - should succeed
  auto status = simple_action_->run({});
  EXPECT_EQ(status, omni_plan::pddl::ActionStatus::SUCCEEDED);
}

TEST_F(YasminActionTest, RunActionWithParameters) {
  // Run the action with parameters
  auto status = move_action_->run({"robot1", "room1", "room2"});
  EXPECT_EQ(status, omni_plan::pddl::ActionStatus::SUCCEEDED);
}

TEST_F(YasminActionTest, ViewerPubDoesNotCreateReferenceCycle) {
  // The viewer stores a shared_ptr to the action it visualizes. If the action
  // also owns the viewer, the reference cycle leaks every action instance.
  auto action = std::make_shared<TestYasminAction>(
      "viewer_yasmin", std::vector<std::pair<std::string, std::string>>{},
      true);
  ASSERT_EQ(action.use_count(), 1);

  EXPECT_EQ(action->run({}), omni_plan::pddl::ActionStatus::SUCCEEDED);

  // Viewer must reference the action without taking ownership.
  EXPECT_EQ(action.use_count(), 1);

  // Destroy the action (and its viewer) while the singleton node is still
  // alive, then tear down the singleton before rclcpp::shutdown() so its
  // executor thread is not left spinning over a dead context.
  action.reset();
  yasmin_ros::YasminNode::destroy_instance();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
