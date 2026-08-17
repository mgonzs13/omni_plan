// Copyright (C) 2026 Miguel Ángel González Santamarta
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

  void TearDown() override { node_.reset(); }

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
// Test: normalize_pddl with name overlapping type substring
// Test: normalize_pddl placeholder map is correct
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

// ==================== Role Key Tests ====================
TEST(CachePlannerRoleKeyTest, ComputeRoleKeys) {
  std::set<omni_plan::pddl::Object> objects;
  objects.insert(omni_plan::pddl::Object("robot1", "robot"));
  objects.insert(omni_plan::pddl::Object("loc1", "location"));
  objects.insert(omni_plan::pddl::Object("loc2", "location"));
  objects.insert(omni_plan::pddl::Object("loc3", "location"));

  auto objs = CachePlanner::group_objects_by_type(objects);

  std::set<omni_plan::pddl::Predicate> facts;
  facts.insert(omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
  facts.insert(omni_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
  facts.insert(omni_plan::pddl::Predicate("connected", {"loc2", "loc3"}));

  std::set<omni_plan::pddl::Predicate> goals;
  goals.insert(omni_plan::pddl::Predicate("at", {"robot1", "loc3"}));

  auto keys = CachePlanner::compute_role_keys(objs, facts, goals);

  // Abstract keys with no alias mapping include only predicate structure
  EXPECT_EQ(keys["robot1"], "at_0_0|at_0_1|");
  EXPECT_EQ(keys["loc1"], "at_1_0|connected_0_0|");
  EXPECT_EQ(keys["loc2"], "connected_0_0|connected_1_0|");
  EXPECT_EQ(keys["loc3"], "at_1_1|connected_1_0|");
}

TEST(CachePlannerRoleKeyTest, ComputeRoleKeysEmptyRole) {
  std::set<omni_plan::pddl::Object> objects;
  objects.insert(omni_plan::pddl::Object("unused", "location"));

  auto objs = CachePlanner::group_objects_by_type(objects);

  std::set<omni_plan::pddl::Predicate> facts;
  std::set<omni_plan::pddl::Predicate> goals;

  auto keys = CachePlanner::compute_role_keys(objs, facts, goals);
  EXPECT_TRUE(keys["unused"].empty());
}

TEST(CachePlannerRoleKeyTest, ComputeRoleKeysDuplicateRoles) {
  // Two locations with identical role: both appear as arg1 in at facts
  std::set<omni_plan::pddl::Object> objects;
  objects.insert(omni_plan::pddl::Object("robot1", "robot"));
  objects.insert(omni_plan::pddl::Object("loc_a", "location"));
  objects.insert(omni_plan::pddl::Object("loc_b", "location"));

  auto objs = CachePlanner::group_objects_by_type(objects);

  std::set<omni_plan::pddl::Predicate> facts;
  facts.insert(omni_plan::pddl::Predicate("at", {"robot1", "loc_a"}));
  facts.insert(omni_plan::pddl::Predicate("at", {"robot1", "loc_b"}));

  std::set<omni_plan::pddl::Predicate> goals;

  auto keys = CachePlanner::compute_role_keys(objs, facts, goals);

  // Both locations have the same abstract role (arg1 in at)
  EXPECT_EQ(keys["loc_a"], keys["loc_b"]);
  EXPECT_EQ(keys["loc_a"], "at_1_0|");
}

// ==================== Structural Key Tests ====================

// Helper: reproduce the two-pass role-key computation that generate_plan
// uses for the structural key: sort by abstract role, build alias map,
// then compute concrete keys with alias-based co-occurrence references.
static std::unordered_map<std::string, std::string>
compute_structural_keys(std::vector<ObjectsByType> &objs,
                        const std::set<omni_plan::pddl::Predicate> &facts,
                        const std::set<omni_plan::pddl::Predicate> &goals) {
  // Pass 1: abstract keys for sorting
  auto abstract = CachePlanner::compute_role_keys(objs, facts, goals);
  for (auto &group : objs) {
    std::sort(group.names.begin(), group.names.end(),
              [&abstract](const std::string &a, const std::string &b) {
                auto it_a = abstract.find(a);
                auto it_b = abstract.find(b);
                const std::string &key_a =
                    (it_a != abstract.end()) ? it_a->second : "";
                const std::string &key_b =
                    (it_b != abstract.end()) ? it_b->second : "";
                if (key_a != key_b)
                  return key_a < key_b;
                return a < b;
              });
  }
  // Alias map from sorted positions
  std::unordered_map<std::string, std::string> alias;
  for (const auto &g : objs) {
    for (size_t i = 0; i < g.names.size(); i++) {
      alias[g.names[i]] = g.type + "_" + std::to_string(i);
    }
  }
  // Pass 2: concrete keys with alias references
  return CachePlanner::compute_role_keys(objs, facts, goals, &alias);
}

static omni_plan::pddl::Domain make_nav_domain() {
  omni_plan::pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_type("item");
  domain.add_predicate(omni_plan::pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(omni_plan::pddl::Predicate("connected", {"?l1", "?l2"}));
  domain.add_predicate(omni_plan::pddl::Predicate("item_at", {"?i", "?l"}));
  return domain;
}

TEST(CachePlannerStructuralKeyTest, ComputeStructuralKeySame) {
  auto domain = make_nav_domain();
  std::string domain_pddl = domain.to_pddl();

  omni_plan::pddl::Problem prob;
  prob.add_object(omni_plan::pddl::Object("robot1", "robot"));
  prob.add_object(omni_plan::pddl::Object("kitchen", "location"));
  prob.add_object(omni_plan::pddl::Object("dining", "location"));
  prob.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "kitchen"}));
  prob.add_fact(omni_plan::pddl::Predicate("connected", {"kitchen", "dining"}));
  prob.add_goal(omni_plan::pddl::Predicate("at", {"robot1", "dining"}));

  auto objs = CachePlanner::group_objects_by_type(prob.get_objects());
  auto keys1 =
      compute_structural_keys(objs, prob.get_facts(), prob.get_goals());
  auto objs2 = CachePlanner::group_objects_by_type(prob.get_objects());
  auto keys2 =
      compute_structural_keys(objs2, prob.get_facts(), prob.get_goals());
  auto k1 =
      CachePlanner::compute_structural_key(domain_pddl, prob, objs, keys1);
  auto k2 =
      CachePlanner::compute_structural_key(domain_pddl, prob, objs2, keys2);

  EXPECT_EQ(k1, k2);
}

TEST(CachePlannerStructuralKeyTest,
     ComputeStructuralKeySameStructureDifferentNames) {
  auto domain = make_nav_domain();
  std::string domain_pddl = domain.to_pddl();

  // Problem A: robot1 in kitchen → dining, connected(kitchen, dining)
  omni_plan::pddl::Problem prob_a;
  prob_a.add_object(omni_plan::pddl::Object("robot1", "robot"));
  prob_a.add_object(omni_plan::pddl::Object("kitchen", "location"));
  prob_a.add_object(omni_plan::pddl::Object("dining", "location"));
  prob_a.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "kitchen"}));
  prob_a.add_fact(
      omni_plan::pddl::Predicate("connected", {"kitchen", "dining"}));
  prob_a.add_goal(omni_plan::pddl::Predicate("at", {"robot1", "dining"}));

  auto objs_a = CachePlanner::group_objects_by_type(prob_a.get_objects());
  auto keys_a =
      compute_structural_keys(objs_a, prob_a.get_facts(), prob_a.get_goals());
  auto key_a =
      CachePlanner::compute_structural_key(domain_pddl, prob_a, objs_a, keys_a);

  // Problem B: r2 in lab → office, connected(lab, office) — structurally
  // identical
  omni_plan::pddl::Problem prob_b;
  prob_b.add_object(omni_plan::pddl::Object("r2", "robot"));
  prob_b.add_object(omni_plan::pddl::Object("lab", "location"));
  prob_b.add_object(omni_plan::pddl::Object("office", "location"));
  prob_b.add_fact(omni_plan::pddl::Predicate("at", {"r2", "lab"}));
  prob_b.add_fact(omni_plan::pddl::Predicate("connected", {"lab", "office"}));
  prob_b.add_goal(omni_plan::pddl::Predicate("at", {"r2", "office"}));

  auto objs_b = CachePlanner::group_objects_by_type(prob_b.get_objects());
  auto keys_b =
      compute_structural_keys(objs_b, prob_b.get_facts(), prob_b.get_goals());
  auto key_b =
      CachePlanner::compute_structural_key(domain_pddl, prob_b, objs_b, keys_b);

  // Two structurally identical problems with different concrete names
  // produce the same structural key — the placeholder-based aliases
  // abstract away the concrete names while preserving role information.
  EXPECT_EQ(key_a, key_b);
}

TEST(CachePlannerStructuralKeyTest, ComputeStructuralKeyDifferentConnectivity) {
  auto domain = make_nav_domain();
  std::string domain_pddl = domain.to_pddl();

  // Problem A: chain — connected(loc1, loc2), connected(loc2, loc3)
  omni_plan::pddl::Problem prob_a;
  prob_a.add_object(omni_plan::pddl::Object("robot1", "robot"));
  prob_a.add_object(omni_plan::pddl::Object("loc1", "location"));
  prob_a.add_object(omni_plan::pddl::Object("loc2", "location"));
  prob_a.add_object(omni_plan::pddl::Object("loc3", "location"));
  prob_a.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
  prob_a.add_fact(omni_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
  prob_a.add_fact(omni_plan::pddl::Predicate("connected", {"loc2", "loc3"}));
  prob_a.add_goal(omni_plan::pddl::Predicate("at", {"robot1", "loc3"}));

  auto objs_a = CachePlanner::group_objects_by_type(prob_a.get_objects());
  auto keys_a =
      compute_structural_keys(objs_a, prob_a.get_facts(), prob_a.get_goals());
  auto key_a =
      CachePlanner::compute_structural_key(domain_pddl, prob_a, objs_a, keys_a);

  // Problem B: star — connected(loc_a, loc_b), connected(loc_a, loc_c)
  // Same type counts and predicate counts, but different connectivity
  omni_plan::pddl::Problem prob_b;
  prob_b.add_object(omni_plan::pddl::Object("robot1", "robot"));
  prob_b.add_object(omni_plan::pddl::Object("loc_a", "location"));
  prob_b.add_object(omni_plan::pddl::Object("loc_b", "location"));
  prob_b.add_object(omni_plan::pddl::Object("loc_c", "location"));
  prob_b.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "loc_a"}));
  prob_b.add_fact(omni_plan::pddl::Predicate("connected", {"loc_a", "loc_b"}));
  prob_b.add_fact(omni_plan::pddl::Predicate("connected", {"loc_a", "loc_c"}));
  prob_b.add_goal(omni_plan::pddl::Predicate("at", {"robot1", "loc_c"}));

  auto objs_b = CachePlanner::group_objects_by_type(prob_b.get_objects());
  auto keys_b =
      compute_structural_keys(objs_b, prob_b.get_facts(), prob_b.get_goals());
  auto key_b =
      CachePlanner::compute_structural_key(domain_pddl, prob_b, objs_b, keys_b);

  EXPECT_NE(key_a, key_b);
}

// Test: same structure but different item-to-location distributions
// produce different structural keys (prevents false cache hit when
// attributes are swapped between structurally identical problems).
TEST(CachePlannerStructuralKeyTest,
     ComputeStructuralKeyDifferentItemDistribution) {
  auto domain = make_nav_domain();
  std::string domain_pddl = domain.to_pddl();

  // Problem A: item1 at kitchen_counter → deliver to table1
  //            item2 at bar_counter0   → deliver to table2
  //            item3 (already carried, not in init, goal at table3)
  omni_plan::pddl::Problem prob_a;
  prob_a.add_object(omni_plan::pddl::Object("robot1", "robot"));
  prob_a.add_object(omni_plan::pddl::Object("item1", "item"));
  prob_a.add_object(omni_plan::pddl::Object("item2", "item"));
  prob_a.add_object(omni_plan::pddl::Object("item3", "item"));
  prob_a.add_object(omni_plan::pddl::Object("kitchen_counter", "location"));
  prob_a.add_object(omni_plan::pddl::Object("bar_counter0", "location"));
  prob_a.add_object(omni_plan::pddl::Object("table1", "location"));
  prob_a.add_object(omni_plan::pddl::Object("table2", "location"));
  prob_a.add_object(omni_plan::pddl::Object("table3", "location"));
  prob_a.add_fact(
      omni_plan::pddl::Predicate("item_at", {"item1", "kitchen_counter"}));
  prob_a.add_fact(
      omni_plan::pddl::Predicate("item_at", {"item2", "bar_counter0"}));
  prob_a.add_fact(
      omni_plan::pddl::Predicate("at", {"robot1", "kitchen_counter"}));
  prob_a.add_goal(omni_plan::pddl::Predicate("item_at", {"item1", "table1"}));
  prob_a.add_goal(omni_plan::pddl::Predicate("item_at", {"item2", "table2"}));
  prob_a.add_goal(omni_plan::pddl::Predicate("item_at", {"item3", "table3"}));

  auto objs_a = CachePlanner::group_objects_by_type(prob_a.get_objects());
  auto keys_a =
      compute_structural_keys(objs_a, prob_a.get_facts(), prob_a.get_goals());
  auto key_a =
      CachePlanner::compute_structural_key(domain_pddl, prob_a, objs_a, keys_a);

  // Problem B: item1 at bar_counter0 → deliver to table2
  //            item2 at kitchen_counter → deliver to table1
  //            item3 unchanged, robot starts at bar_counter0
  omni_plan::pddl::Problem prob_b;
  prob_b.add_object(omni_plan::pddl::Object("robot1", "robot"));
  prob_b.add_object(omni_plan::pddl::Object("item1", "item"));
  prob_b.add_object(omni_plan::pddl::Object("item2", "item"));
  prob_b.add_object(omni_plan::pddl::Object("item3", "item"));
  prob_b.add_object(omni_plan::pddl::Object("kitchen_counter", "location"));
  prob_b.add_object(omni_plan::pddl::Object("bar_counter0", "location"));
  prob_b.add_object(omni_plan::pddl::Object("table1", "location"));
  prob_b.add_object(omni_plan::pddl::Object("table2", "location"));
  prob_b.add_object(omni_plan::pddl::Object("table3", "location"));
  prob_b.add_fact(
      omni_plan::pddl::Predicate("item_at", {"item1", "bar_counter0"}));
  prob_b.add_fact(
      omni_plan::pddl::Predicate("item_at", {"item2", "kitchen_counter"}));
  prob_b.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "bar_counter0"}));
  prob_b.add_goal(omni_plan::pddl::Predicate("item_at", {"item1", "table2"}));
  prob_b.add_goal(omni_plan::pddl::Predicate("item_at", {"item2", "table1"}));
  prob_b.add_goal(omni_plan::pddl::Predicate("item_at", {"item3", "table3"}));

  auto objs_b = CachePlanner::group_objects_by_type(prob_b.get_objects());
  auto keys_b =
      compute_structural_keys(objs_b, prob_b.get_facts(), prob_b.get_goals());
  auto key_b =
      CachePlanner::compute_structural_key(domain_pddl, prob_b, objs_b, keys_b);

  // Different distribution → different concrete keys → cache miss
  EXPECT_NE(key_a, key_b);

  // Problem C: same distribution as A but with different object names.
  // Should produce the same structural key as A.
  omni_plan::pddl::Problem prob_c;
  prob_c.add_object(omni_plan::pddl::Object("robot2", "robot"));
  prob_c.add_object(omni_plan::pddl::Object("itemX", "item"));
  prob_c.add_object(omni_plan::pddl::Object("itemY", "item"));
  prob_c.add_object(omni_plan::pddl::Object("itemZ", "item"));
  prob_c.add_object(omni_plan::pddl::Object("counter_x", "location"));
  prob_c.add_object(omni_plan::pddl::Object("counter_y", "location"));
  prob_c.add_object(omni_plan::pddl::Object("table_x", "location"));
  prob_c.add_object(omni_plan::pddl::Object("table_y", "location"));
  prob_c.add_object(omni_plan::pddl::Object("table_z", "location"));
  prob_c.add_fact(
      omni_plan::pddl::Predicate("item_at", {"itemX", "counter_x"}));
  prob_c.add_fact(
      omni_plan::pddl::Predicate("item_at", {"itemY", "counter_y"}));
  prob_c.add_fact(omni_plan::pddl::Predicate("at", {"robot2", "counter_x"}));
  prob_c.add_goal(omni_plan::pddl::Predicate("item_at", {"itemX", "table_x"}));
  prob_c.add_goal(omni_plan::pddl::Predicate("item_at", {"itemY", "table_y"}));
  prob_c.add_goal(omni_plan::pddl::Predicate("item_at", {"itemZ", "table_z"}));

  auto objs_c = CachePlanner::group_objects_by_type(prob_c.get_objects());
  auto keys_c =
      compute_structural_keys(objs_c, prob_c.get_facts(), prob_c.get_goals());
  auto key_c =
      CachePlanner::compute_structural_key(domain_pddl, prob_c, objs_c, keys_c);

  // Same attribute-pattern as A → same concrete role keys after alias
  EXPECT_EQ(key_a, key_c);
}

// ==================== Mock Planner for Cache Integration Tests
class MockPlanner : public omni_plan::Planner {
public:
  mutable int generate_call_count_ = 0;
  std::string plan_output_;

  MockPlanner() : Planner() {
    plan_output_ = "0.000: (move robot1 loc1 loc2) [10.000]\n";
  }

  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &domain,
                const omni_plan::pddl::Problem & /*problem*/) const override {
    generate_call_count_++;
    return this->parse_plan(domain, plan_output_);
  }

  bool has_solution(const std::string &str) const override {
    return !str.empty();
  }
};

// Test subclass that can inject a wrapped planner without pluginlib
class TestableCachePlanner : public CachePlanner {
public:
  void inject_wrapped_planner(std::shared_ptr<omni_plan::Planner> p) {
    wrapped_planner_ = std::move(p);
  }
};

class CachePlannerCacheTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_cache_node");
    planner_ = std::make_unique<TestableCachePlanner>();
    planner_->load_ros_parameters(node_);
    // Inject mock planner directly (bypasses pluginlib)
    planner_->inject_wrapped_planner(mock_);

    // Build domain with move action
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
  }

  void TearDown() override { node_.reset(); }

  // Build a simple problem: robot at from → to with a single connected edge
  omni_plan::pddl::Problem make_problem(const std::string &robot,
                                        const std::string &from,
                                        const std::string &to) const {
    omni_plan::pddl::Problem prob;
    prob.add_object(omni_plan::pddl::Object(robot, "robot"));
    prob.add_object(omni_plan::pddl::Object(from, "location"));
    prob.add_object(omni_plan::pddl::Object(to, "location"));
    prob.add_fact(omni_plan::pddl::Predicate("at", {robot, from}));
    prob.add_fact(omni_plan::pddl::Predicate("connected", {from, to}));
    prob.add_goal(omni_plan::pddl::Predicate("at", {robot, to}));
    return prob;
  }

  // Build a structurally different problem: chain with 3 locations
  omni_plan::pddl::Problem make_chain_problem() const {
    omni_plan::pddl::Problem prob;
    prob.add_object(omni_plan::pddl::Object("robot1", "robot"));
    prob.add_object(omni_plan::pddl::Object("loc1", "location"));
    prob.add_object(omni_plan::pddl::Object("loc2", "location"));
    prob.add_object(omni_plan::pddl::Object("loc3", "location"));
    prob.add_fact(omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    prob.add_fact(omni_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
    prob.add_fact(omni_plan::pddl::Predicate("connected", {"loc2", "loc3"}));
    prob.add_goal(omni_plan::pddl::Predicate("at", {"robot1", "loc3"}));
    return prob;
  }

  std::unique_ptr<TestableCachePlanner> planner_;
  std::shared_ptr<MockPlanner> mock_ = std::make_shared<MockPlanner>();
  std::shared_ptr<rclcpp::Node> node_;
  omni_plan::pddl::Domain domain_;
};

// Test: first call is a cache miss, second call with same problem is exact hit
TEST_F(CachePlannerCacheTest, ExactCacheHit) {
  auto prob_a = make_problem("robot1", "loc1", "loc2");

  // First call: cache miss
  auto plan1 = planner_->generate_plan(domain_, prob_a);
  EXPECT_TRUE(plan1.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Second call with same problem: exact cache hit (no planner call)
  auto plan2 = planner_->generate_plan(domain_, prob_a);
  EXPECT_TRUE(plan2.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Both plans must be identical
  EXPECT_EQ(plan1.get_raw_output(), plan2.get_raw_output());
}

// Test: structurally identical problem with different names triggers
// structural cache hit and produces an adapted plan
TEST_F(CachePlannerCacheTest, StructuralCacheHit) {
  auto prob_a = make_problem("robot1", "loc1", "loc2");

  // First call: cache miss — mock planner called once
  auto plan1 = planner_->generate_plan(domain_, prob_a);
  EXPECT_TRUE(plan1.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Second call: structurally identical but different object names
  auto prob_b = make_problem("robot2", "loc3", "loc4");
  auto plan2 = planner_->generate_plan(domain_, prob_b);
  EXPECT_TRUE(plan2.has_solution());
  // No additional planner call — structural cache hit
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Verify adapted plan has new names
  EXPECT_EQ(plan2.size(), 1u);
  auto params2 = plan2.get_action_params(0);
  ASSERT_EQ(params2.size(), 3u);
  EXPECT_EQ(params2[0], "robot2");
  EXPECT_EQ(params2[1], "loc3");
  EXPECT_EQ(params2[2], "loc4");
}

// Test: structurally different problem generates a cache miss
TEST_F(CachePlannerCacheTest, StructuralMiss) {
  auto prob_a = make_problem("robot1", "loc1", "loc2");

  // First call: cache miss
  planner_->generate_plan(domain_, prob_a);
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Second call: structurally different (3 locations chain)
  auto prob_c = make_chain_problem();
  planner_->generate_plan(domain_, prob_c);
  EXPECT_EQ(mock_->generate_call_count_, 2);

  // Third call: exact hit for the first problem
  planner_->generate_plan(domain_, prob_a);
  EXPECT_EQ(mock_->generate_call_count_, 2);

  // Fourth call: exact hit for the second problem
  planner_->generate_plan(domain_, prob_c);
  EXPECT_EQ(mock_->generate_call_count_, 2);
}

// ==================== Swap Name Mapping Test ====================
TEST_F(CachePlannerTest, BuildNameMappingSwap) {
  // Simulate cached problem: robot1 at kitchen → bedroom
  std::unordered_map<std::string, std::string> old_placeholder_to_original = {
      {"__obj_robot_0__", "robot1"},
      {"__obj_location_0__", "kitchen"},
      {"__obj_location_1__", "bedroom"},
  };

  // New problem: robot1 at bedroom → kitchen (swap of old)
  std::set<omni_plan::pddl::Object> new_objects;
  new_objects.insert(omni_plan::pddl::Object("robot1", "robot"));
  new_objects.insert(omni_plan::pddl::Object("bedroom", "location"));
  new_objects.insert(omni_plan::pddl::Object("kitchen", "location"));

  // Role-sort new objects (same as generate_plan does before
  // build_name_mapping)
  std::set<omni_plan::pddl::Predicate> new_facts;
  new_facts.insert(omni_plan::pddl::Predicate("at", {"robot1", "bedroom"}));
  std::set<omni_plan::pddl::Predicate> new_goals;
  new_goals.insert(omni_plan::pddl::Predicate("at", {"robot1", "kitchen"}));

  auto new_objs = CachePlanner::group_objects_by_type(new_objects);
  auto role_keys =
      CachePlanner::compute_role_keys(new_objs, new_facts, new_goals);
  for (auto &group : new_objs) {
    std::sort(group.names.begin(), group.names.end(),
              [&role_keys](const std::string &a, const std::string &b) {
                auto it_a = role_keys.find(a);
                auto it_b = role_keys.find(b);
                const std::string &key_a =
                    (it_a != role_keys.end()) ? it_a->second : "";
                const std::string &key_b =
                    (it_b != role_keys.end()) ? it_b->second : "";
                if (key_a != key_b)
                  return key_a < key_b;
                return a < b;
              });
  }

  auto mapping =
      CachePlanner::build_name_mapping(old_placeholder_to_original, new_objs);

  EXPECT_EQ(mapping["kitchen"], "bedroom");
  EXPECT_EQ(mapping["bedroom"], "kitchen");
  EXPECT_EQ(mapping["robot1"], "robot1");
}

// ==================== Relevance & Object Filtering Tests
//
// Domain: move action (at, connected preconds) + item_at predicate
// that is NEVER used in any action — should be filtered as irrelevant.

class CachePlannerRelevanceTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_relevance_node");
    planner_ = std::make_unique<TestableCachePlanner>();
    planner_->load_ros_parameters(node_);
    planner_->inject_wrapped_planner(mock_);

    domain_.add_requirement("strips");
    domain_.add_requirement("typing");
    domain_.add_type("location");
    domain_.add_type("robot");
    domain_.add_type("item");
    domain_.add_predicate(omni_plan::pddl::Predicate("at", {"?r", "?l"}));
    domain_.add_predicate(
        omni_plan::pddl::Predicate("connected", {"?l1", "?l2"}));
    domain_.add_predicate(omni_plan::pddl::Predicate("item_at", {"?i", "?l"}));

    auto move_action = std::make_shared<MockAction>(
        "move",
        std::vector<std::pair<std::string, std::string>>{
            {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}});
    move_action->add_condition(omni_plan::pddl::Type::START, "at",
                               {"?r", "?from"});
    move_action->add_condition(omni_plan::pddl::Type::START, "connected",
                               {"?from", "?to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"?r", "?from"},
                            true);
    domain_.add_action(move_action);
  }

  void TearDown() override { node_.reset(); }

  omni_plan::pddl::Problem make_base(const std::string &robot,
                                     const std::string &from,
                                     const std::string &to) const {
    omni_plan::pddl::Problem prob;
    prob.add_object(omni_plan::pddl::Object(robot, "robot"));
    prob.add_object(omni_plan::pddl::Object(from, "location"));
    prob.add_object(omni_plan::pddl::Object(to, "location"));
    prob.add_fact(omni_plan::pddl::Predicate("at", {robot, from}));
    prob.add_fact(omni_plan::pddl::Predicate("connected", {from, to}));
    prob.add_goal(omni_plan::pddl::Predicate("at", {robot, to}));
    return prob;
  }

  omni_plan::pddl::Problem
  make_with_item_at(const std::string &robot, const std::string &from,
                    const std::string &to, const std::string &item,
                    const std::string &item_loc) const {
    auto prob = make_base(robot, from, to);
    prob.add_object(omni_plan::pddl::Object(item, "item"));
    prob.add_fact(omni_plan::pddl::Predicate("item_at", {item, item_loc}));
    return prob;
  }

  std::unique_ptr<TestableCachePlanner> planner_;
  std::shared_ptr<MockPlanner> mock_ = std::make_shared<MockPlanner>();
  std::shared_ptr<rclcpp::Node> node_;
  omni_plan::pddl::Domain domain_;
};

// An item_at fact with a new item object is filtered as irrelevant, so the
// problem produces the same structural key as the base problem → cache hit.
TEST_F(CachePlannerRelevanceTest, IrrelevantPredicateFiltered) {
  auto prob_a = make_base("robot1", "loc1", "loc2");

  auto plan1 = planner_->generate_plan(domain_, prob_a);
  EXPECT_TRUE(plan1.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  auto prob_b = make_with_item_at("robot1", "loc1", "loc2", "item1", "loc1");
  auto plan2 = planner_->generate_plan(domain_, prob_b);
  EXPECT_TRUE(plan2.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  EXPECT_EQ(plan2.size(), 1u);
  auto params = plan2.get_action_params(0);
  EXPECT_EQ(params[0], "robot1");
  EXPECT_EQ(params[1], "loc1");
  EXPECT_EQ(params[2], "loc2");
}

// Different content in irrelevant predicates + different names in relevant
// objects: the structural key matches (item_at ignored) and names are adapted.
TEST_F(CachePlannerRelevanceTest, IrrelevantPredicateDifferentNames) {
  mock_->plan_output_ = "0.000: (move robot1 loc1 loc2) [10.000]\n";
  auto prob_a = make_with_item_at("robot1", "loc1", "loc2", "item1", "loc1");

  auto plan1 = planner_->generate_plan(domain_, prob_a);
  EXPECT_TRUE(plan1.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Problem B: same relevant structure, different names everywhere, different
  // irrelevant item_at content — item_at is irrelevant so key should match.
  auto prob_b =
      make_with_item_at("robot2", "lab", "office", "itemX", "warehouse");
  auto plan2 = planner_->generate_plan(domain_, prob_b);
  EXPECT_TRUE(plan2.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  EXPECT_EQ(plan2.size(), 1u);
  auto params = plan2.get_action_params(0);
  EXPECT_EQ(params[0], "robot2");
  EXPECT_EQ(params[1], "lab");
  EXPECT_EQ(params[2], "office");
}

// An object declared in :objects that never appears in any predicate is
// filtered by the empty-key filter → structural hit despite different type
// counts.
TEST_F(CachePlannerRelevanceTest, UnreferencedObjectFiltered) {
  auto prob_a = make_base("robot1", "loc1", "loc2");

  auto plan1 = planner_->generate_plan(domain_, prob_a);
  EXPECT_TRUE(plan1.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Same problem but with an extra location that no predicate references.
  auto prob_b = make_base("robot1", "loc1", "loc2");
  prob_b.add_object(omni_plan::pddl::Object("extra", "location"));

  auto plan2 = planner_->generate_plan(domain_, prob_b);
  EXPECT_TRUE(plan2.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  EXPECT_EQ(plan2.size(), 1u);
  auto params = plan2.get_action_params(0);
  EXPECT_EQ(params[0], "robot1");
  EXPECT_EQ(params[1], "loc1");
  EXPECT_EQ(params[2], "loc2");
}

// Both an irrelevant predicate AND unreferenced objects together → still a
// structural cache hit with correct name adaptation.
TEST_F(CachePlannerRelevanceTest, CombinedIrrelevant) {
  mock_->plan_output_ = "0.000: (move robot1 loc1 loc2) [10.000]\n";
  auto prob_a = make_base("robot1", "loc1", "loc2");

  auto plan1 = planner_->generate_plan(domain_, prob_a);
  EXPECT_TRUE(plan1.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  // Problem B: new names, an irrelevant item_at, AND two extra unreferenced
  // objects of different types.  None should break the structural key.
  omni_plan::pddl::Problem prob_b;
  prob_b.add_object(omni_plan::pddl::Object("r2", "robot"));
  prob_b.add_object(omni_plan::pddl::Object("kitchen", "location"));
  prob_b.add_object(omni_plan::pddl::Object("bedroom", "location"));
  prob_b.add_fact(omni_plan::pddl::Predicate("at", {"r2", "kitchen"}));
  prob_b.add_fact(
      omni_plan::pddl::Predicate("connected", {"kitchen", "bedroom"}));
  prob_b.add_goal(omni_plan::pddl::Predicate("at", {"r2", "bedroom"}));
  // Irrelevant parts:
  prob_b.add_object(omni_plan::pddl::Object("itemX", "item"));
  prob_b.add_fact(omni_plan::pddl::Predicate("item_at", {"itemX", "kitchen"}));
  prob_b.add_object(omni_plan::pddl::Object("unused_item", "item"));
  prob_b.add_object(omni_plan::pddl::Object("extra_room", "location"));

  auto plan2 = planner_->generate_plan(domain_, prob_b);
  EXPECT_TRUE(plan2.has_solution());
  EXPECT_EQ(mock_->generate_call_count_, 1);

  EXPECT_EQ(plan2.size(), 1u);
  auto params = plan2.get_action_params(0);
  EXPECT_EQ(params[0], "r2");
  EXPECT_EQ(params[1], "kitchen");
  EXPECT_EQ(params[2], "bedroom");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
