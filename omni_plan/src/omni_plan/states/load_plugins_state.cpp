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
    this->set_description("Load the plugins for the PDDL manager, planner, "
                          "plan validator and actions.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::SUCCEED,
                                  "Plugins loaded successfully.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::ABORT,
                                  "Failed to load plugins.");
    this->add_input_key("pddl_manager.plugin",
                        "The plugin name for the PDDL manager.");
    this->add_input_key("planner.plugin", "The plugin name for the planner.");
    this->add_input_key("plan_validator.plugin",
                        "The plugin name for the plan validator.");
    this->add_input_key("actions_plugins", "The plugin names for the actions.");
    this->add_output_key("pddl_manager", "The loaded PDDL manager plugin.");
    this->add_output_key("planner", "The loaded planner plugin.");
    this->add_output_key("plan_validator", "The loaded plan validator plugin.");
    this->add_output_key("actions", "The loaded action plugins.");

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
    if (!pddl_manager_plugin.empty()) {
      try {
        auto pddl_manager = std::shared_ptr<omni_plan::PddlManager>(
            this->pddl_manager_state_loader_.createUnmanagedInstance(
                pddl_manager_plugin));
        pddl_manager->load_ros_parameters(
            yasmin_ros::YasminNode::get_instance());
        blackboard->set<std::shared_ptr<omni_plan::PddlManager>>("pddl_manager",
                                                                 pddl_manager);
      } catch (const std::exception &e) {
        YASMIN_LOG_ERROR("Failed to load PddlManager plugin '%s': %s",
                         pddl_manager_plugin.c_str(), e.what());
        return yasmin_ros::basic_outcomes::ABORT;
      }
    }

    // Load Planner plugin
    std::string planner_plugin = blackboard->get<std::string>("planner.plugin");

    if (!planner_plugin.empty()) {
      try {
        auto planner = std::shared_ptr<omni_plan::Planner>(
            this->planner_state_loader_.createUnmanagedInstance(
                planner_plugin));
        planner->load_ros_parameters(yasmin_ros::YasminNode::get_instance());
        blackboard->set<std::shared_ptr<omni_plan::Planner>>("planner",
                                                             planner);
      } catch (const std::exception &e) {
        YASMIN_LOG_ERROR("Failed to load Planner plugin '%s': %s",
                         planner_plugin.c_str(), e.what());
        return yasmin_ros::basic_outcomes::ABORT;
      }
    }

    // Load PlanValidator plugin
    std::string plan_validator_plugin =
        blackboard->get<std::string>("plan_validator.plugin");

    if (!plan_validator_plugin.empty()) {
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
    }

    // Load Action plugins
    auto actions_plugins =
        blackboard->get<std::vector<std::string>>("actions_plugins");

    std::unordered_map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        actions;

    omni_plan_msgs::msg::ActionInfoArray info_msg;

    for (const auto &action_plugin : actions_plugins) {
      if (action_plugin.empty()) {
        continue;
      }

      try {
        auto plugin = std::shared_ptr<omni_plan::pddl::Action>(
            this->action_state_loader_.createUnmanagedInstance(action_plugin));
        plugin->load_ros_parameters(yasmin_ros::YasminNode::get_instance());

        actions[plugin->get_name()] = plugin;
        info_msg.actions.push_back(plugin->to_msg());
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