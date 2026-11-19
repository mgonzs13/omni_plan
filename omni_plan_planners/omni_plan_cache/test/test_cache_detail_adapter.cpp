// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan_cache/detail/plan_adapter.hpp"

using namespace omni_plan;
using namespace omni_plan_cache;
using namespace omni_plan_cache::detail;

namespace {

class MockAction : public pddl::Action {
public:
  MockAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params)
      : Action(name, params) {}
  pddl::ActionStatus run(const std::vector<std::string> &) override {
    return pddl::ActionStatus::SUCCEEDED;
  }
  void cancel() override {}
};

} // namespace

TEST(PlanAdapterTest, BuildNameMappingAlignsPlaceholders) {
  std::unordered_map<std::string, std::string> old = {
      {"__obj_robot_0__", "robot1"},
      {"__obj_location_0__", "kitchen"},
      {"__obj_location_1__", "dining_room"}};
  std::vector<ObjectsByType> new_objects = {{"robot", {"r2"}},
                                            {"location", {"lab", "office"}}};

  auto mapping = PlanAdapter::build_name_mapping(old, new_objects);
  EXPECT_EQ(mapping["robot1"], "r2");
  EXPECT_EQ(mapping["kitchen"], "lab");
  EXPECT_EQ(mapping["dining_room"], "office");
}

TEST(PlanAdapterTest, AdaptRenamesParamsAndKeepsRawOutput) {
  auto action = std::make_shared<MockAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}});
  CachedPlanData cached;
  cached.plan.set_has_solution(true);
  cached.plan.set_raw_output("0.000: (move robot1 kitchen dining) [10.000]\n");
  cached.plan.add_action(action, {"robot1", "kitchen", "dining"}, 0.0f);

  auto adapted = PlanAdapter::adapt(
      cached, {{"robot1", "r2"}, {"kitchen", "lab"}, {"dining", "office"}});

  EXPECT_TRUE(adapted.has_solution());
  ASSERT_EQ(adapted.size(), 1u);
  const auto params = adapted.get_action_params(0);
  EXPECT_EQ(params[0], "r2");
  EXPECT_EQ(params[1], "lab");
  EXPECT_EQ(params[2], "office");
  EXPECT_FALSE(adapted.get_raw_output().empty());
}
