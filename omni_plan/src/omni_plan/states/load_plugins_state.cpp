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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl_manager.hpp"
#include "omni_plan/plan_validator.hpp"
#include "omni_plan/planner.hpp"

#include "omni_plan_msgs/msg/action_info_array.hpp"

class LoadPluginsState : public yasmin::State {

public:
  LoadPluginsState()
      : yasmin::State({
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::ABORT,
        }),
        pddl_manager_state_loader_("omni_plan", "omni_plan::PddlManager"),
        planner_state_loader_("omni_plan", "omni_plan::Planner"),
        plan_validator_state_loader_("omni_plan", "omni_plan::PlanValidator"),
        action_state_loader_("omni_plan", "omni_plan::pddl::Action") {

    auto node = yasmin_ros::YasminNode::get_instance();
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    this->actions_info_pub_ =
        node->create_publisher<omni_plan_msgs::msg::ActionInfoArray>(
            "/omni_plan/actions_info", qos);
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) {

    // Load PddlManager plugin
    std::string pddl_manager_plugin =
        blackboard->get<std::string>("pddl_manager.plugin");
    try {
      auto pddl_manager = std::shared_ptr<omni_plan::PddlManager>(
          this->pddl_manager_state_loader_.createUnmanagedInstance(
              pddl_manager_plugin));
      pddl_manager->load_ros_parameters(yasmin_ros::YasminNode::get_instance());
      blackboard->set<std::shared_ptr<omni_plan::PddlManager>>("pddl_manager",
                                                               pddl_manager);
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to load PddlManager plugin '%s': %s",
                       pddl_manager_plugin.c_str(), e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    // Load Planner plugin
    std::string planner_plugin = blackboard->get<std::string>("planner.plugin");

    try {
      auto planner = std::shared_ptr<omni_plan::Planner>(
          this->planner_state_loader_.createUnmanagedInstance(planner_plugin));
      planner->load_ros_parameters(yasmin_ros::YasminNode::get_instance());
      blackboard->set<std::shared_ptr<omni_plan::Planner>>("planner", planner);
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to load Planner plugin '%s': %s",
                       planner_plugin.c_str(), e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    // Load PlanValidator plugin
    std::string plan_validator_plugin =
        blackboard->get<std::string>("plan_validator.plugin");

    try {
      auto plan_validator = std::shared_ptr<omni_plan::PlanValidator>(
          this->plan_validator_state_loader_.createUnmanagedInstance(
              plan_validator_plugin));
      plan_validator->load_ros_parameters(
          yasmin_ros::YasminNode::get_instance());
      blackboard->set<std::shared_ptr<omni_plan::PlanValidator>>(
          "plan_validator", plan_validator);
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to load PlanValidator plugin '%s': %s",
                       plan_validator_plugin.c_str(), e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }

    // Load Action plugins
    std::vector<std::string> actions_plugins =
        blackboard->get<std::vector<std::string>>("actions_plugins");

    std::unordered_map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        actions;

    for (const auto &action_plugin : actions_plugins) {
      if (action_plugin.empty()) {
        continue;
      }

      try {
        auto action = std::shared_ptr<omni_plan::pddl::Action>(
            this->action_state_loader_.createUnmanagedInstance(action_plugin));
        action->load_ros_parameters(yasmin_ros::YasminNode::get_instance());
        actions[action->get_name()] = action;
      } catch (const std::exception &e) {
        YASMIN_LOG_ERROR("Failed to create Action plugin instance '%s': %s",
                         action_plugin.c_str(), e.what());
        return yasmin_ros::basic_outcomes::ABORT;
      }
    }

    blackboard->set<std::unordered_map<
        std::string, std::shared_ptr<omni_plan::pddl::Action>>>("actions",
                                                                actions);

    // Publish latched action info so monitors can inspect available actions
    omni_plan_msgs::msg::ActionInfoArray info_msg;
    for (const auto &kv : actions) {
      info_msg.actions.push_back(kv.second->to_msg());
    }
    this->actions_info_pub_->publish(info_msg);

    return yasmin_ros::basic_outcomes::SUCCEED;
  }

private:
  pluginlib::ClassLoader<omni_plan::PddlManager> pddl_manager_state_loader_;
  pluginlib::ClassLoader<omni_plan::Planner> planner_state_loader_;
  pluginlib::ClassLoader<omni_plan::PlanValidator> plan_validator_state_loader_;
  pluginlib::ClassLoader<omni_plan::pddl::Action> action_state_loader_;
  rclcpp::Publisher<omni_plan_msgs::msg::ActionInfoArray>::SharedPtr
      actions_info_pub_;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LoadPluginsState, yasmin::State)