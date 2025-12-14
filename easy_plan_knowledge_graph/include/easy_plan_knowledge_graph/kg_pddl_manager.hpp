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

#ifndef EASY_PLAN__KG_PDDL_MANAGER_HPP__
#define EASY_PLAN__KG_PDDL_MANAGER_HPP__

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include <knowledge_graph/knowledge_graph.hpp>
#include <knowledge_graph_msgs/msg/graph_update.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "easy_plan/pddl/expression.hpp"
#include "easy_plan/pddl_manager.hpp"

namespace easy_plan_knowledge_graph {

class KgPddlManager : public easy_plan::PddlManager {
public:
  KgPddlManager(std::shared_ptr<knowledge_graph::KnowledgeGraph> kg = nullptr);

  using PddlManager::get_pddl;

  std::pair<std::string, std::string>
  get_pddl(std::set<std::string> actions_types,
           std::set<std::string> actions_predicates,
           std::set<std::string> actions_pddl) const override;

  bool has_goals() const override;

  void apply_effect(easy_plan::pddl::Effect exp) override;

  void undo_effect(easy_plan::pddl::Effect exp) override;

private:
  void graph_update_callback(
      const knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg);

  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
  mutable std::mutex goal_mutex_;
  mutable std::condition_variable goal_cv_;
  rclcpp::Subscription<knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr
      update_sub_;
};

} // namespace easy_plan_knowledge_graph
#endif // EASY_PLAN__KG_PDDL_MANAGER_HPP__