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
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/cache_planner.hpp"

using namespace omni_plan_cache;

class MockAction : public omni_plan::pddl::Action {
public:
  MockAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params), cancel_called_(false) {}

  omni_plan::pddl::ActionStatus run(const std::vector<std::string> &) override {
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { cancel_called_ = true; }

  bool cancel_called_;
};

class CachePlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_cache_node");
    planner_ = std::make_unique<CachePlanner>();
    planner_->load_ros_parameters(node_);

    // Build domain
    domain_.add_requirement("strips");
    domain_.add_requirement("typing");
    domain_.add_type("location");
    domain_.add_type("robot");
    domain_.add_predicate(omni_plan::pddl::Predicate("at", {"?r", "?l"}));
    domain_.add_predicate(
        omni_plan::pddl::Predicate("connected", {"?l1", "?l2"}));

    std::vector<std::pair<std::string, std::string>> params = {
        {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    move_action->add_condition(omni_plan::pddl::Type::START, "at",
                               {"?r", "?from"});
    move_action->add_condition(omni_plan::pddl::Type::START, "connected",
                               {"?from", "?to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?from"},
                            true);
    domain_.add_action(move_action);

    // Build problem
    problem_.add_object(omni_plan::pddl::Object("robot1", "robot"));
    problem_.add_object(omni_plan::pddl::Object("loc1", "location"));
    problem_.add_object(omni_plan::pddl::Object("loc2", "location"));
    problem_.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    problem_.add_fact(
        omni_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
    problem_.add_goal(omni_plan::pddl::Predicate("at", {"robot1", "loc2"}));
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::unique_ptr<CachePlanner> planner_;
  std::shared_ptr<rclcpp::Node> node_;
  omni_plan::pddl::Domain domain_;
  omni_plan::pddl::Problem problem_;
};

// Test: Constructor
TEST_F(CachePlannerTest, ConstructorCreatesPlanner) {
  EXPECT_NE(planner_, nullptr);
}

// Test: sha256 produces consistent results
TEST_F(CachePlannerTest, Sha256IsConsistent) {
  std::string hash1 = CachePlanner::sha256("hello");
  std::string hash2 = CachePlanner::sha256("hello");
  EXPECT_EQ(hash1, hash2);
  EXPECT_NE(hash1, CachePlanner::sha256("world"));
}

// Test: hash is 64 hex chars (SHA-256)
TEST_F(CachePlannerTest, Sha256Length) {
  std::string hash = CachePlanner::sha256("test");
  EXPECT_EQ(hash.size(), 64u);
}

// Test: group_objects_by_type groups correctly
TEST_F(CachePlannerTest, GroupObjectsByType) {
  std::set<omni_plan::pddl::Object> objects;
  objects.insert(omni_plan::pddl::Object("r1", "robot"));
  objects.insert(omni_plan::pddl::Object("r2", "robot"));
  objects.insert(omni_plan::pddl::Object("loc1", "location"));

  auto groups = CachePlanner::group_objects_by_type(objects);

  EXPECT_EQ(groups.size(), 2u);

  for (const auto &g : groups) {
    if (g.type == "robot") {
      EXPECT_EQ(g.names.size(), 2u);
    } else if (g.type == "location") {
      EXPECT_EQ(g.names.size(), 1u);
    }
  }
}

// Test: normalize_pddl replaces object names with placeholders (boundary-aware)
TEST_F(CachePlannerTest, NormalizePddl) {
  std::string pddl = "(at robot1 loc1) (at robot1 loc2)";
  std::set<omni_plan::pddl::Object> objects;
  objects.insert(omni_plan::pddl::Object("robot1", "robot"));
  objects.insert(omni_plan::pddl::Object("loc1", "location"));
  objects.insert(omni_plan::pddl::Object("loc2", "location"));

  auto objs = CachePlanner::group_objects_by_type(objects);
  std::unordered_map<std::string, std::string> placeholder_map;
  std::string normalized =
      CachePlanner::normalize_pddl(pddl, objs, placeholder_map);

  EXPECT_NE(normalized.find("__obj_robot_0__"), std::string::npos);
  EXPECT_NE(normalized.find("__obj_location_0__"), std::string::npos);
  EXPECT_NE(normalized.find("__obj_location_1__"), std::string::npos);
  EXPECT_EQ(normalized.find("robot1"), std::string::npos);
  EXPECT_EQ(normalized.find("loc1"), std::string::npos);
}

// Test: normalize_pddl with name overlapping type substring
TEST_F(CachePlannerTest, NormalizePddlNameSubstringOfType) {
  // Object "loc" of type "location" — "loc" appears in type name "location"
  std::string pddl = "(at robot loc) (connected loc location_a)";
  std::set<omni_plan::pddl::Object> objects;
  objects.insert(omni_plan::pddl::Object("robot", "robot"));
  objects.insert(omni_plan::pddl::Object("loc", "location"));
  objects.insert(omni_plan::pddl::Object("location_a", "location"));

  auto objs = CachePlanner::group_objects_by_type(objects);
  std::unordered_map<std::string, std::string> placeholder_map;
  std::string normalized =
      CachePlanner::normalize_pddl(pddl, objs, placeholder_map);

  // "loc" and "location_a" should be replaced; "loc" should NOT match inside
  // placeholders
  EXPECT_NE(normalized.find("__obj_location_0__"), std::string::npos);
  EXPECT_NE(normalized.find("__obj_location_1__"), std::string::npos);
  EXPECT_EQ(normalized.find(" loc "), std::string::npos);
}

// Test: normalize_pddl placeholder map is correct
TEST_F(CachePlannerTest, NormalizePddlPlaceholderMap) {
  std::string pddl = "(at robot1 loc1)";
  std::set<omni_plan::pddl::Object> objects;
  objects.insert(omni_plan::pddl::Object("robot1", "robot"));
  objects.insert(omni_plan::pddl::Object("loc1", "location"));

  auto objs = CachePlanner::group_objects_by_type(objects);
  std::unordered_map<std::string, std::string> placeholder_map;
  CachePlanner::normalize_pddl(pddl, objs, placeholder_map);

  EXPECT_EQ(placeholder_map["__obj_robot_0__"], "robot1");
  EXPECT_EQ(placeholder_map["__obj_location_0__"], "loc1");
}

// Test: generate_plan with no wrapped planner throws
TEST_F(CachePlannerTest, NoWrappedPlannerThrows) {
  EXPECT_THROW(planner_->generate_plan(domain_, problem_), std::runtime_error);
}

// Test: build_name_mapping produces correct mapping
TEST_F(CachePlannerTest, BuildNameMapping) {
  std::unordered_map<std::string, std::string> old_placeholder_to_original = {
      {"__obj_robot_0__", "robot1"},
      {"__obj_location_0__", "kitchen"},
      {"__obj_location_1__", "dining_room"},
  };

  std::set<omni_plan::pddl::Object> new_objects;
  new_objects.insert(omni_plan::pddl::Object("r2", "robot"));
  new_objects.insert(omni_plan::pddl::Object("lab", "location"));
  new_objects.insert(omni_plan::pddl::Object("office", "location"));

  auto new_objs = CachePlanner::group_objects_by_type(new_objects);
  auto mapping =
      CachePlanner::build_name_mapping(old_placeholder_to_original, new_objs);

  EXPECT_EQ(mapping["robot1"], "r2");
  EXPECT_EQ(mapping["kitchen"], "lab");
  EXPECT_EQ(mapping["dining_room"], "office");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
