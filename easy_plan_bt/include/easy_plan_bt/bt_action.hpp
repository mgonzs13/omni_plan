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

#ifndef EASY_PLAN_BT__BT_ACTION_HPP_
#define EASY_PLAN_BT__BT_ACTION_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>

#include "easy_plan/pddl/action.hpp"

namespace easy_plan_bt {

/**
 * @class BtAction
 * @brief Represents a Behavior Tree action in the easy_plan framework.
 */
class BtAction : public easy_plan::pddl::Action {
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
   * @return The status of the action execution (SUCCEED, CANCEL, ABORT).
   */
  easy_plan::pddl::ActionStatus
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
  /// @brief Tick rate for the Behavior Tree execution.
  int tick_rate_;
  /// @brief Flag indicating if the action has been canceled.
  std::atomic_bool is_canceled_;

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
  std::unique_ptr<BT::PublisherZMQ> groot_monitor_;
};

} // namespace easy_plan_bt
#endif // EASY_PLAN_BT__BT_ACTION_HPP_