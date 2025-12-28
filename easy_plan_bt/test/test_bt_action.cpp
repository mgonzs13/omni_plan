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

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easy_plan_bt/bt_action.hpp"

using namespace easy_plan_bt;

/**
 * @brief Test fixture for BtAction tests
 */
class BtActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a simple behavior tree XML for testing with absolute path
    bt_xml_path_ = std::filesystem::absolute("tree.xml").string();
    std::ofstream bt_file(bt_xml_path_);
    bt_file << R"(<root BTCPP_format="4" >
   <BehaviorTree ID="Main">
     <ReactiveSequence>
       <AlwaysSuccess name="success"/>
     </ReactiveSequence>
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
  EXPECT_EQ(status, easy_plan::pddl::ActionStatus::SUCCEED);
}

TEST_F(BtActionTest, RunActionAbortInvalidBTFile) {
  // Remove the test BT file to simulate invalid file path
  std::remove(bt_xml_path_.c_str());

  // Run the action
  auto status = test_action_->run({"value1", "value2"});

  // Verify that the action aborted due to invalid BT file
  EXPECT_EQ(status, easy_plan::pddl::ActionStatus::ABORT);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
