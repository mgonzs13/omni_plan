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
#include <thread>
#include <vector>

#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan_knowledge_base/knowledge_base.hpp"
#include "easy_plan_knowledge_base/knowledge_base_exceptions.hpp"

using namespace easy_plan_knowledge_base;
using namespace easy_plan::pddl;

class KnowledgeBaseTest : public ::testing::Test {
protected:
  void SetUp() override { kb_ = std::make_unique<KnowledgeBase>(); }

  void TearDown() override { kb_->clear(); }

  std::unique_ptr<KnowledgeBase> kb_;
};

// ==================== Type Tests ====================

TEST_F(KnowledgeBaseTest, AddTypeSuccess) {
  EXPECT_TRUE(kb_->add_type("robot"));
  EXPECT_TRUE(kb_->has_type("robot"));
}

TEST_F(KnowledgeBaseTest, AddTypeDuplicate) {
  EXPECT_TRUE(kb_->add_type("robot"));
  EXPECT_FALSE(kb_->add_type("robot")); // Duplicate should return false
}

TEST_F(KnowledgeBaseTest, RemoveTypeSuccess) {
  kb_->add_type("robot");
  EXPECT_TRUE(kb_->remove_type("robot"));
  EXPECT_FALSE(kb_->has_type("robot"));
}

TEST_F(KnowledgeBaseTest, RemoveTypeNotFound) {
  EXPECT_FALSE(kb_->remove_type("nonexistent"));
}

TEST_F(KnowledgeBaseTest, RemoveTypeCascadesToObjects) {
  // When removing a type, all objects of that type should be removed
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("robot2", "robot"));
  kb_->add_object(Object("loc1", "location"));

  kb_->remove_type("robot");

  EXPECT_FALSE(kb_->has_type("robot"));
  EXPECT_FALSE(kb_->has_object(Object("robot1", "robot")));
  EXPECT_FALSE(kb_->has_object(Object("robot2", "robot")));
  // Location object should remain
  EXPECT_TRUE(kb_->has_object(Object("loc1", "location")));
}

TEST_F(KnowledgeBaseTest, RemoveTypeCascadesToFactsAndGoals) {
  // When removing a type, facts/goals referencing those objects should be
  // removed
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_type("?l1");
  kb_->add_type("?l2");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  kb_->add_predicate(Predicate("connected", {"?l1", "?l2"}));
  kb_->add_fact(Predicate("at", {"robot1", "loc1"}));
  kb_->add_goal(Predicate("at", {"robot1", "loc2"}));
  kb_->add_fact(Predicate("connected", {"loc1", "loc2"}));

  kb_->remove_type("robot");

  // Facts/goals referencing robot1 should be removed
  EXPECT_FALSE(kb_->has_fact(Predicate("at", {"robot1", "loc1"})));
  EXPECT_FALSE(kb_->has_goal(Predicate("at", {"robot1", "loc2"})));
  // Facts not referencing robot objects should remain
  EXPECT_TRUE(kb_->has_fact(Predicate("connected", {"loc1", "loc2"})));
}

TEST_F(KnowledgeBaseTest, RemoveTypeCascadesToPredicates) {
  // When removing a type used in predicate arguments, predicates should be
  // removed
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  kb_->add_predicate(Predicate("battery", {"?r"}));

  // Remove the type used in predicates
  kb_->remove_type("?r");

  // Predicates using that type should be removed
  EXPECT_FALSE(kb_->has_predicate(Predicate("at", {"?r", "?l"})));
  EXPECT_FALSE(kb_->has_predicate(Predicate("battery", {"?r"})));
}

TEST_F(KnowledgeBaseTest, GetTypesEmpty) {
  auto types = kb_->get_types();
  EXPECT_TRUE(types.empty());
}

TEST_F(KnowledgeBaseTest, GetTypesMultiple) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_type("waypoint");

  auto types = kb_->get_types();
  EXPECT_EQ(types.size(), 3u);
}

// ==================== Object Tests ====================

TEST_F(KnowledgeBaseTest, AddObjectSuccess) {
  kb_->add_type("robot");
  Object obj("robot1", "robot");
  EXPECT_TRUE(kb_->add_object(obj));
  EXPECT_TRUE(kb_->has_object(obj));
}

TEST_F(KnowledgeBaseTest, AddObjectAutoCreatesType) {
  // This test is no longer valid - adding object without type should throw
  EXPECT_FALSE(kb_->has_type("robot"));
  Object obj("robot1", "robot");
  EXPECT_THROW(kb_->add_object(obj), TypeNotFoundException);
}

TEST_F(KnowledgeBaseTest, AddObjectWithoutTypeThrows) {
  // Verify that adding an object without its type throws exception
  Object obj("robot1", "robot");
  EXPECT_THROW(kb_->add_object(obj), TypeNotFoundException);
}

TEST_F(KnowledgeBaseTest, AddObjectDuplicate) {
  kb_->add_type("robot");
  Object obj("robot1", "robot");
  EXPECT_TRUE(kb_->add_object(obj));
  EXPECT_FALSE(kb_->add_object(obj));
}

TEST_F(KnowledgeBaseTest, RemoveObjectSuccess) {
  kb_->add_type("robot");
  Object obj("robot1", "robot");
  kb_->add_object(obj);
  EXPECT_TRUE(kb_->remove_object(obj));
  EXPECT_FALSE(kb_->has_object(obj));
}

TEST_F(KnowledgeBaseTest, RemoveObjectCascadesToFactsAndGoals) {
  // When removing an object, facts/goals referencing it should be removed
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_predicate(Predicate("at", {"robot", "location"}));
  kb_->add_predicate(Predicate("connected", {"location", "location"}));
  kb_->add_fact(Predicate("at", {"robot1", "loc1"}));
  kb_->add_goal(Predicate("at", {"robot1", "loc2"}));
  kb_->add_fact(Predicate("connected", {"loc1", "loc2"}));

  kb_->remove_object(Object("robot1", "robot"));

  // Facts/goals referencing robot1 should be removed
  EXPECT_FALSE(kb_->has_fact(Predicate("at", {"robot1", "loc1"})));
  EXPECT_FALSE(kb_->has_goal(Predicate("at", {"robot1", "loc2"})));
  // Facts not referencing robot1 should remain
  EXPECT_TRUE(kb_->has_fact(Predicate("connected", {"loc1", "loc2"})));
}

TEST_F(KnowledgeBaseTest, GetObjectsEmpty) {
  auto objects = kb_->get_objects();
  EXPECT_TRUE(objects.empty());
}

TEST_F(KnowledgeBaseTest, GetObjectsByType) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("robot2", "robot"));
  kb_->add_object(Object("loc1", "location"));

  auto robots = kb_->get_objects_by_type("robot");
  EXPECT_EQ(robots.size(), 2u);

  auto locations = kb_->get_objects_by_type("location");
  EXPECT_EQ(locations.size(), 1u);
}

// ==================== Predicate Tests ====================

TEST_F(KnowledgeBaseTest, AddPredicateSuccess) {
  kb_->add_type("?r");
  kb_->add_type("?l");
  Predicate pred("at", {"?r", "?l"});
  EXPECT_TRUE(kb_->add_predicate(pred));
  EXPECT_TRUE(kb_->has_predicate(pred));
}

TEST_F(KnowledgeBaseTest, AddPredicateWithoutTypeThrows) {
  // Verify that adding a predicate without argument types throws exception
  Predicate pred("at", {"?r", "?l"});
  EXPECT_THROW(kb_->add_predicate(pred), TypeNotFoundException);
}

TEST_F(KnowledgeBaseTest, AddPredicateDuplicate) {
  kb_->add_type("?r");
  kb_->add_type("?l");
  Predicate pred("at", {"?r", "?l"});
  EXPECT_TRUE(kb_->add_predicate(pred));
  EXPECT_FALSE(kb_->add_predicate(pred));
}

TEST_F(KnowledgeBaseTest, RemovePredicateSuccess) {
  kb_->add_type("?r");
  kb_->add_type("?l");
  Predicate pred("at", {"?r", "?l"});
  kb_->add_predicate(pred);
  EXPECT_TRUE(kb_->remove_predicate(pred));
  EXPECT_FALSE(kb_->has_predicate(pred));
}

TEST_F(KnowledgeBaseTest, RemovePredicateCascadesToFactsAndGoals) {
  // When removing a predicate definition, facts/goals using that predicate
  // should be removed
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("robot2", "robot"));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_object(Object("loc3", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_type("?l1");
  kb_->add_type("?l2");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  kb_->add_predicate(Predicate("connected", {"?l1", "?l2"}));
  kb_->add_fact(Predicate("at", {"robot1", "loc1"}));
  kb_->add_fact(Predicate("at", {"robot2", "loc2"}));
  kb_->add_goal(Predicate("at", {"robot1", "loc3"}));
  kb_->add_fact(Predicate("connected", {"loc1", "loc2"}));

  kb_->remove_predicate(Predicate("at", {"?r", "?l"}));

  // Facts/goals with predicate name "at" should be removed
  EXPECT_FALSE(kb_->has_fact(Predicate("at", {"robot1", "loc1"})));
  EXPECT_FALSE(kb_->has_fact(Predicate("at", {"robot2", "loc2"})));
  EXPECT_FALSE(kb_->has_goal(Predicate("at", {"robot1", "loc3"})));
  // Facts with other predicate names should remain
  EXPECT_TRUE(kb_->has_fact(Predicate("connected", {"loc1", "loc2"})));
}

TEST_F(KnowledgeBaseTest, GetPredicatesEmpty) {
  auto predicates = kb_->get_predicates();
  EXPECT_TRUE(predicates.empty());
}

// ==================== Fact Tests ====================

TEST_F(KnowledgeBaseTest, AddFactSuccess) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate fact("at", {"robot1", "loc1"});
  EXPECT_TRUE(kb_->add_fact(fact));
  EXPECT_TRUE(kb_->has_fact(fact));
}

TEST_F(KnowledgeBaseTest, AddFactWithoutPredicateThrows) {
  // Verify that adding a fact without predicate definition throws exception
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc1", "location"));
  Predicate fact("at", {"robot1", "loc1"});
  EXPECT_THROW(kb_->add_fact(fact), PredicateNotFoundException);
}

TEST_F(KnowledgeBaseTest, AddFactWithoutObjectThrows) {
  // Verify that adding a fact without object throws exception
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate fact("at", {"robot1", "loc1"});
  EXPECT_THROW(kb_->add_fact(fact), ObjectNotFoundException);
}

TEST_F(KnowledgeBaseTest, AddFactDuplicate) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate fact("at", {"robot1", "loc1"});
  EXPECT_TRUE(kb_->add_fact(fact));
  EXPECT_FALSE(kb_->add_fact(fact));
}

TEST_F(KnowledgeBaseTest, RemoveFactSuccess) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate fact("at", {"robot1", "loc1"});
  kb_->add_fact(fact);
  EXPECT_TRUE(kb_->remove_fact(fact));
  EXPECT_FALSE(kb_->has_fact(fact));
}

TEST_F(KnowledgeBaseTest, GetFactsEmpty) {
  auto facts = kb_->get_facts();
  EXPECT_TRUE(facts.empty());
}

TEST_F(KnowledgeBaseTest, GetFactsByName) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("robot2", "robot"));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_type("?l1");
  kb_->add_type("?l2");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  kb_->add_predicate(Predicate("connected", {"?l1", "?l2"}));
  kb_->add_fact(Predicate("at", {"robot1", "loc1"}));
  kb_->add_fact(Predicate("at", {"robot2", "loc2"}));
  kb_->add_fact(Predicate("connected", {"loc1", "loc2"}));

  auto at_facts = kb_->get_facts_by_name("at");
  EXPECT_EQ(at_facts.size(), 2u);

  auto connected_facts = kb_->get_facts_by_name("connected");
  EXPECT_EQ(connected_facts.size(), 1u);
}

// ==================== Goal Tests ====================

TEST_F(KnowledgeBaseTest, AddGoalSuccess) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate goal("at", {"robot1", "loc2"});
  EXPECT_TRUE(kb_->add_goal(goal));
  EXPECT_TRUE(kb_->has_goal(goal));
}

TEST_F(KnowledgeBaseTest, AddGoalWithoutPredicateThrows) {
  // Verify that adding a goal without predicate definition throws exception
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc2", "location"));
  Predicate goal("at", {"robot1", "loc2"});
  EXPECT_THROW(kb_->add_goal(goal), PredicateNotFoundException);
}

TEST_F(KnowledgeBaseTest, AddGoalWithoutObjectThrows) {
  // Verify that adding a goal without object throws exception
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate goal("at", {"robot1", "loc2"});
  EXPECT_THROW(kb_->add_goal(goal), ObjectNotFoundException);
}

TEST_F(KnowledgeBaseTest, AddGoalDuplicate) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate goal("at", {"robot1", "loc2"});
  EXPECT_TRUE(kb_->add_goal(goal));
  EXPECT_FALSE(kb_->add_goal(goal));
}

TEST_F(KnowledgeBaseTest, RemoveGoalSuccess) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  Predicate goal("at", {"robot1", "loc2"});
  kb_->add_goal(goal);
  EXPECT_TRUE(kb_->remove_goal(goal));
  EXPECT_FALSE(kb_->has_goal(goal));
}

TEST_F(KnowledgeBaseTest, GetGoalsEmpty) {
  auto goals = kb_->get_goals();
  EXPECT_TRUE(goals.empty());
}

TEST_F(KnowledgeBaseTest, HasGoalsTrue) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  kb_->add_goal(Predicate("at", {"robot1", "loc2"}));
  EXPECT_TRUE(kb_->has_goals());
}

TEST_F(KnowledgeBaseTest, HasGoalsFalse) { EXPECT_FALSE(kb_->has_goals()); }

// ==================== Clear Tests ====================

TEST_F(KnowledgeBaseTest, ClearAll) {
  kb_->add_type("robot");
  kb_->add_type("location");
  kb_->add_object(Object("robot1", "robot"));
  kb_->add_type("?r");
  kb_->add_type("?l");
  kb_->add_predicate(Predicate("at", {"?r", "?l"}));
  kb_->add_object(Object("loc1", "location"));
  kb_->add_object(Object("loc2", "location"));
  kb_->add_fact(Predicate("at", {"robot1", "loc1"}));
  kb_->add_goal(Predicate("at", {"robot1", "loc2"}));

  kb_->clear();

  EXPECT_TRUE(kb_->get_types().empty());
  EXPECT_TRUE(kb_->get_objects().empty());
  EXPECT_TRUE(kb_->get_predicates().empty());
  EXPECT_TRUE(kb_->get_facts().empty());
  EXPECT_TRUE(kb_->get_goals().empty());
}

// ==================== Thread Safety Tests ====================

TEST_F(KnowledgeBaseTest, ConcurrentAccess) {
  std::vector<std::thread> threads;

  // Add multiple types concurrently
  for (int i = 0; i < 10; ++i) {
    threads.emplace_back(
        [this, i]() { kb_->add_type("type" + std::to_string(i)); });
  }

  for (auto &t : threads) {
    t.join();
  }

  EXPECT_EQ(kb_->get_types().size(), 10u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
