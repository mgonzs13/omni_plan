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

#ifndef OMNI_PLAN_BT__BT_ACTION_HPP_
#define OMNI_PLAN_BT__BT_ACTION_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#if defined(BTV3)
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#else
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/utils/shared_library.h>
#endif

#include "omni_plan/pddl/action.hpp"

namespace omni_plan_bt {

#if defined(BTV3)
using BTMonitor = BT::PublisherZMQ;
#else
using BTMonitor = BT::Groot2Publisher;
#endif

/**
 * @class BtAction
 * @brief Represents a Behavior Tree action in the omni_plan framework.
 */
class BtAction : public omni_plan::pddl::Action {
public:
  /**
   * @brief Constructs a BtAction with a given name and optional parameters.
   * @param name The name of the action.
   * @param params The parameters of the action (default is an empty vector).
   */
  BtAction(const std::string &name,
           const std::vector<std::pair<std::string, std::string>> &params = {},
           const std::string &default_bt_file_path = "tree.xml");

  /**
   * @brief Virtual destructor for the BtAction class.
   */
  virtual ~BtAction() = default;

  /**
   * @brief Executes the Behavior Tree action.
   * @param params The parameters for the action execution.
   * @return The status of the action execution (SUCCEEDED, CANCELED, ABORTED).
   */
  omni_plan::pddl::ActionStatus
  run(const std::vector<std::string> &params) override;

  /**
   * @brief Cancels the execution of the Behavior Tree action.
   */
  void cancel() override;

protected:
  /**
   * @brief Loads data into the blackboard for backward execution.
   */
  virtual void load_data_in_blackboard();

  /// @brief Behavior Tree blackboard.
  BT::Blackboard::Ptr blackboard_;

private:
  /**
   * @brief Loads the Behavior Tree from the specified XML file.
   */
  void load_tree();

  /// @brief Behavior Tree instance.
  std::shared_ptr<BT::Tree> tree_;
  /// @brief Flag indicating if the action has been canceled.
  std::atomic_bool is_canceled_{false};

  //// @brief Default path to the Behavior Tree XML file.
  std::string default_bt_file_path_;
  /// @brief Path to the Behavior Tree XML file.
  std::string bt_file_path_;
  /// @brief List of plugins to load for the Behavior Tree.
  std::vector<std::string> plugins_;
  /// @brief Behavior Tree factory.
  BT::BehaviorTreeFactory bt_factory_;

  /// @brief Flag to enable Groot monitoring.
  bool enable_groot_monitoring_;
  /// @brief Maximum messages per second for Groot monitoring.
  int max_msg_per_second_;
  /// @brief Publisher port for Groot monitoring.
  int publisher_port_;
  /// @brief Server port for Groot monitoring.
  int server_port_;
  /// @brief Behavior Tree ZMQ publisher for Groot monitoring.
  std::unique_ptr<BTMonitor> groot_monitor_;
};

} // namespace omni_plan_bt
#endif // OMNI_PLAN_BT__BT_ACTION_HPP_