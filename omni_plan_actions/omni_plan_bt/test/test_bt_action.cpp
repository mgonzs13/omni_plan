// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_bt/bt_action.hpp"

using namespace omni_plan_bt;

/**
 * @brief Test fixture for BtAction tests
 */
class BtActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a simple behavior tree XML for testing with absolute path
    bt_xml_path_ = std::filesystem::absolute("tree.xml").string();
    std::ofstream bt_file(bt_xml_path_);
    bt_file << R"(<root main_tree_to_execute="Main">
   <BehaviorTree ID="Main">
     <Sequence>
       <AlwaysSuccess name="success"/>
     </Sequence>
   </BehaviorTree>
 </root>)";
    bt_file.close();

    test_action_ = std::make_shared<BtAction>(
        "test_bt", std::vector<std::pair<std::string, std::string>>{
                       {"param1", "type1"},
                       {"param2", "type2"},
                   });

    node_ = std::make_shared<rclcpp::Node>("bt_action_test_node");
    test_action_->load_ros_parameters(node_);
  }

  void TearDown() override {
    // Clean up test file
    std::remove(bt_xml_path_.c_str());
  }

  std::shared_ptr<BtAction> test_action_;
  std::string bt_xml_path_;
  rclcpp::Node::SharedPtr node_;
};

TEST_F(BtActionTest, ConstructAction) { EXPECT_NE(test_action_, nullptr); }

TEST_F(BtActionTest, RunActionSuccess) {

  // Run the action
  auto status = test_action_->run({"value1", "value2"});

  // Verify that the action succeeded
  EXPECT_EQ(status, omni_plan::pddl::ActionStatus::SUCCEEDED);
}

TEST_F(BtActionTest, RunAfterCancelRunsTreeAgain) {
  // A canceled instance is cached and reused by later plans, so a single
  // cancellation must not poison the instance for subsequent runs.
  EXPECT_EQ(test_action_->run({"value1", "value2"}),
            omni_plan::pddl::ActionStatus::SUCCEEDED);

  test_action_->cancel();

  EXPECT_EQ(test_action_->run({"value1", "value2"}),
            omni_plan::pddl::ActionStatus::SUCCEEDED);
}

TEST_F(BtActionTest, RunActionAbortInvalidBTFile) {
  // Remove the test BT file to simulate invalid file path
  std::remove(bt_xml_path_.c_str());
  test_action_->load_ros_parameters(node_);

  // Run the action
  auto status = test_action_->run({"value1", "value2"});

  // Verify that the action aborted due to invalid BT file
  EXPECT_EQ(status, omni_plan::pddl::ActionStatus::ABORTED);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
