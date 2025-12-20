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

#ifndef EASY_PLAN__KG_PDDL_MANAGER_HPP_
#define EASY_PLAN__KG_PDDL_MANAGER_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph/knowledge_graph.hpp"

#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/expression.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/pddl_manager.hpp"

namespace easy_plan_knowledge_graph {

class KgPddlManager : public easy_plan::PddlManager {
public:
  KgPddlManager(bool add_callback = true);

  using PddlManager::get_pddl;

  std::pair<easy_plan::pddl::Domain, easy_plan::pddl::Problem>
  get_pddl() const override;

  bool has_goals() const override;

  bool
  predicate_exists(const easy_plan::pddl::Predicate &predicate) const override;

  bool
  predicate_is_goal(const easy_plan::pddl::Predicate &predicate) const override;

  void apply_effect(const easy_plan::pddl::Effect &exp) override;

private:
  void graph_callback(
      const std::string &operation, const std::string &element_type,
      const std::vector<std::variant<knowledge_graph::graph::Node,
                                     knowledge_graph::graph::Edge>> &elements);

  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
  mutable std::mutex goal_mutex_;
  mutable std::condition_variable goal_cv_;
};

} // namespace easy_plan_knowledge_graph
#endif // EASY_PLAN__KG_PDDL_MANAGER_HPP_