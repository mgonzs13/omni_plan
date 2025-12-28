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

#include "easy_plan_yasmin/yasmin_action.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

using namespace easy_plan_yasmin;

/**
 * @brief Concrete implementation of YasminAction for testing
 */
class TestYasminAction : public YasminAction {
public:
  TestYasminAction(
      const std::string &name,
      const std::vector<std::pair<std::string, std::string>> &params = {})
      : YasminAction(name, params) {

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
  EXPECT_EQ(status, easy_plan::pddl::ActionStatus::SUCCEED);
}

TEST_F(YasminActionTest, RunActionWithParameters) {
  // Run the action with parameters
  auto status = move_action_->run({"robot1", "room1", "room2"});
  EXPECT_EQ(status, easy_plan::pddl::ActionStatus::SUCCEED);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
