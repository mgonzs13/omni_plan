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

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_yasmin/yasmin_factory_action.hpp"

using namespace omni_plan_yasmin;

/**
 * @brief Test fixture for YasminFactoryAction tests
 */
class YasminFactoryActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a simple state machine XML for testing with absolute path
    sm_xml_path_ = std::filesystem::absolute("state_machine.xml").string();
    std::ofstream sm_file(sm_xml_path_);
    sm_file << R"(<StateMachine outcomes="succeeded">
    <State name="State1" type="cpp" class="yasmin_factory/TestSimpleState">
        <Transition from="outcome1" to="State1"/>
        <Transition from="outcome2" to="succeeded"/>
    </State>
</StateMachine>)";
    sm_file.close();

    simple_action_ = std::make_shared<YasminFactoryAction>("simple_factory");

    node_ = std::make_shared<rclcpp::Node>("sm_action_test_node");
    node_->declare_parameter("simple_factory_action.enable_viewer_pub", false);
    simple_action_->load_ros_parameters(node_);
  }

  void TearDown() override {
    // Clean up test file
    std::remove(sm_xml_path_.c_str());
  }
  std::shared_ptr<YasminFactoryAction> simple_action_;

  std::string sm_xml_path_;
  rclcpp::Node::SharedPtr node_;
};

TEST_F(YasminFactoryActionTest, ConstructAction) {
  EXPECT_NE(simple_action_, nullptr);
}

TEST_F(YasminFactoryActionTest, RunActionSuccess) {

  // Run the action
  auto status = simple_action_->run({});

  // Verify that the action succeeded
  EXPECT_EQ(status, omni_plan::pddl::ActionStatus::SUCCEEDED);
}

TEST_F(YasminFactoryActionTest, RunActionAbortInvalidSMFile) {
  // Remove the test SM file to simulate invalid file path
  std::remove(sm_xml_path_.c_str());
  simple_action_->load_ros_parameters(node_);

  // Run the action
  auto status = simple_action_->run({"value1", "value2"});

  // Verify that the action aborted due to invalid SM file
  EXPECT_EQ(status, omni_plan::pddl::ActionStatus::ABORTED);
}

TEST_F(YasminFactoryActionTest, MalformedXmlWithViewerEnabled_DoesNotCrash) {
  // Write a malformed XML file that exists but cannot be parsed.
  const std::string malformed_path =
      std::filesystem::absolute("malformed_state_machine.xml").string();
  {
    std::ofstream malformed_file(malformed_path);
    malformed_file << "<StateMachine outcomes=\"succeeded\"><not-closed>";
  }

  // Enable the viewer so that a null state machine would previously be passed
  // to YasminViewerPub and cause a segfault.
  node_->declare_parameter("malformed_factory_action.enable_viewer_pub", true);
  node_->declare_parameter("malformed_factory_action.state_machine_xml",
                           malformed_path);

  auto action = std::make_shared<YasminFactoryAction>("malformed_factory");
  action->load_ros_parameters(node_);

  // The malformed XML must be reported as ABORTED, not crash the process.
  EXPECT_EQ(action->run({}), omni_plan::pddl::ActionStatus::ABORTED);

  action.reset();
  std::remove(malformed_path.c_str());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
