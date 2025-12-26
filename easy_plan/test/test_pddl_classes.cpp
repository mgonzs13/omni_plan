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
#include <set>
#include <string>
#include <vector>

#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/pddl/timing_predicate.hpp"

using namespace easy_plan::pddl;

// ==================== Object Tests ====================

class ObjectTest : public ::testing::Test {};

TEST_F(ObjectTest, ConstructorSetsNameAndType) {
  Object obj("robot1", "robot");

  EXPECT_EQ(obj.get_name(), "robot1");
  EXPECT_EQ(obj.get_type(), "robot");
}

TEST_F(ObjectTest, LessThanOperatorByName) {
  Object obj1("alpha", "type_a");
  Object obj2("beta", "type_a");

  EXPECT_TRUE(obj1 < obj2);
  EXPECT_FALSE(obj2 < obj1);
}

TEST_F(ObjectTest, LessThanOperatorByType) {
  Object obj1("robot1", "type_a");
  Object obj2("robot1", "type_b");

  EXPECT_TRUE(obj1 < obj2);
  EXPECT_FALSE(obj2 < obj1);
}

TEST_F(ObjectTest, EqualObjectsNotLessThan) {
  Object obj1("robot1", "robot");
  Object obj2("robot1", "robot");

  EXPECT_FALSE(obj1 < obj2);
  EXPECT_FALSE(obj2 < obj1);
}

TEST_F(ObjectTest, ObjectsInSet) {
  std::set<Object> objects;

  objects.insert(Object("robot1", "robot"));
  objects.insert(Object("robot2", "robot"));
  objects.insert(Object("robot1", "robot")); // Duplicate

  EXPECT_EQ(objects.size(), 2u);
}

// ==================== Predicate Tests ====================

class PredicateTest : public ::testing::Test {};

TEST_F(PredicateTest, ConstructorWithDefaults) {
  Predicate pred("at", {"robot1", "loc1"});

  EXPECT_EQ(pred.get_name(), "at");
  EXPECT_EQ(pred.get_args().size(), 2u);
  EXPECT_FALSE(pred.is_negated());
}

TEST_F(PredicateTest, ConstructorWithNegation) {
  Predicate pred("holding", {"robot1", "item1"}, true);

  EXPECT_EQ(pred.get_name(), "holding");
  EXPECT_TRUE(pred.is_negated());
}

TEST_F(PredicateTest, ConstructorEmptyArgs) {
  Predicate pred("done");

  EXPECT_EQ(pred.get_name(), "done");
  EXPECT_EQ(pred.get_args().size(), 0u);
}

TEST_F(PredicateTest, SetNegation) {
  Predicate pred("at", {"robot1", "loc1"});

  EXPECT_FALSE(pred.is_negated());
  pred.set_negation(true);
  EXPECT_TRUE(pred.is_negated());
  pred.set_negation(false);
  EXPECT_FALSE(pred.is_negated());
}

TEST_F(PredicateTest, ToPddlAsFact) {
  Predicate pred("at", {"robot1", "loc1"});

  std::string pddl = pred.to_pddl(true);

  EXPECT_EQ(pddl, "(at robot1 loc1)");
}

TEST_F(PredicateTest, ToPddlAsFactNegated) {
  Predicate pred("holding", {"robot1", "item1"}, true);

  std::string pddl = pred.to_pddl(true);

  EXPECT_TRUE(pddl.find("(not") != std::string::npos);
  EXPECT_TRUE(pddl.find("(holding robot1 item1)") != std::string::npos);
}

TEST_F(PredicateTest, ToPddlAsPredicateDefinition) {
  Predicate pred("at", {"robot", "location"});

  std::string pddl = pred.to_pddl(false);

  // Should include variable notation
  EXPECT_TRUE(pddl.find("?") != std::string::npos);
  EXPECT_TRUE(pddl.find("at") != std::string::npos);
}

TEST_F(PredicateTest, LessThanOperatorByName) {
  Predicate pred1("at", {"a", "b"});
  Predicate pred2("holding", {"a", "b"});

  EXPECT_TRUE(pred1 < pred2);
  EXPECT_FALSE(pred2 < pred1);
}

TEST_F(PredicateTest, LessThanOperatorByArgs) {
  Predicate pred1("at", {"a", "b"});
  Predicate pred2("at", {"c", "d"});

  EXPECT_TRUE(pred1 < pred2);
  EXPECT_FALSE(pred2 < pred1);
}

TEST_F(PredicateTest, LessThanOperatorByNegation) {
  Predicate pred1("at", {"a", "b"}, false);
  Predicate pred2("at", {"a", "b"}, true);

  EXPECT_TRUE(pred1 < pred2);
  EXPECT_FALSE(pred2 < pred1);
}

TEST_F(PredicateTest, PredicatesInSet) {
  std::set<Predicate> predicates;

  predicates.insert(Predicate("at", {"robot1", "loc1"}));
  predicates.insert(Predicate("holding", {"robot1", "item1"}));
  predicates.insert(Predicate("at", {"robot1", "loc1"})); // Duplicate

  EXPECT_EQ(predicates.size(), 2u);
}

// ==================== TimingPredicate Tests ====================

class TimingPredicateTest : public ::testing::Test {};

TEST_F(TimingPredicateTest, ConstructorStartType) {
  TimingPredicate pred(Type::START, "at", {"r", "l"});

  EXPECT_EQ(pred.get_type(), Type::START);
  EXPECT_EQ(pred.get_name(), "at");
  EXPECT_FALSE(pred.is_negated());
}

TEST_F(TimingPredicateTest, ConstructorOverAllType) {
  TimingPredicate pred(Type::OVER_ALL, "moving", {"r"});

  EXPECT_EQ(pred.get_type(), Type::OVER_ALL);
}

TEST_F(TimingPredicateTest, ConstructorEndType) {
  TimingPredicate pred(Type::END, "done", {"r"}, true);

  EXPECT_EQ(pred.get_type(), Type::END);
  EXPECT_TRUE(pred.is_negated());
}

TEST_F(TimingPredicateTest, ToPddlStart) {
  TimingPredicate pred(Type::START, "at", {"r", "l"});

  std::string pddl = pred.to_pddl();

  EXPECT_TRUE(pddl.find("at start") != std::string::npos);
  EXPECT_TRUE(pddl.find("(at ?r ?l)") != std::string::npos);
}

TEST_F(TimingPredicateTest, ToPddlOverAll) {
  TimingPredicate pred(Type::OVER_ALL, "moving", {"r"});

  std::string pddl = pred.to_pddl();

  EXPECT_TRUE(pddl.find("over all") != std::string::npos);
}

TEST_F(TimingPredicateTest, ToPddlEnd) {
  TimingPredicate pred(Type::END, "finished", {"r"});

  std::string pddl = pred.to_pddl();

  EXPECT_TRUE(pddl.find("at end") != std::string::npos);
}

TEST_F(TimingPredicateTest, ToPddlNegated) {
  TimingPredicate pred(Type::START, "at", {"r", "l"}, true);

  std::string pddl = pred.to_pddl();

  EXPECT_TRUE(pddl.find("(not") != std::string::npos);
}

// ==================== Domain Tests ====================

class DomainTest : public ::testing::Test {};

TEST_F(DomainTest, AddRequirement) {
  Domain domain;

  domain.add_requirement("strips");
  domain.add_requirement("typing");

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find(":strips") != std::string::npos);
  EXPECT_TRUE(pddl.find(":typing") != std::string::npos);
}

TEST_F(DomainTest, AddDuplicateRequirement) {
  Domain domain;

  domain.add_requirement("strips");
  domain.add_requirement("strips"); // Duplicate

  std::string pddl = domain.to_pddl();

  // Should only appear once (using set internally)
  size_t count = 0;
  size_t pos = 0;
  while ((pos = pddl.find(":strips", pos)) != std::string::npos) {
    ++count;
    pos += 7;
  }
  EXPECT_EQ(count, 1u);
}

TEST_F(DomainTest, AddType) {
  Domain domain;

  domain.add_type("robot");
  domain.add_type("location");

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(:types") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("location") != std::string::npos);
}

TEST_F(DomainTest, AddPredicate) {
  Domain domain;

  domain.add_predicate(Predicate("at", {"robot", "location"}));
  domain.add_predicate(Predicate("holding", {"robot", "item"}));

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(:predicates") != std::string::npos);
}

TEST_F(DomainTest, EmptyDomain) {
  Domain domain;

  std::string pddl = domain.to_pddl();

  EXPECT_TRUE(pddl.find("(define (domain easy_plan_domain)") !=
              std::string::npos);
  EXPECT_TRUE(pddl.find("(:requirements)") != std::string::npos);
}

TEST_F(DomainTest, DomainWithoutTypes) {
  Domain domain;
  domain.add_requirement("strips");

  std::string pddl = domain.to_pddl();

  // Types section should not appear if no types
  EXPECT_TRUE(pddl.find("(:types") == std::string::npos);
}

TEST_F(DomainTest, DomainWithoutPredicates) {
  Domain domain;
  domain.add_requirement("strips");
  domain.add_type("robot");

  std::string pddl = domain.to_pddl();

  // Predicates section should not appear if no predicates
  EXPECT_TRUE(pddl.find("(:predicates") == std::string::npos);
}

// ==================== Problem Tests ====================

class ProblemTest : public ::testing::Test {};

TEST_F(ProblemTest, AddObject) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc1", "location"));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(:objects") != std::string::npos);
  EXPECT_TRUE(pddl.find("robot1 - robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("loc1 - location") != std::string::npos);
}

TEST_F(ProblemTest, AddFact) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc1", "location"));
  problem.add_fact(Predicate("at", {"robot1", "loc1"}));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(:init") != std::string::npos);
  EXPECT_TRUE(pddl.find("(at robot1 loc1)") != std::string::npos);
}

TEST_F(ProblemTest, AddGoal) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc2", "location"));
  problem.add_goal(Predicate("at", {"robot1", "loc2"}));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(:goal") != std::string::npos);
  EXPECT_TRUE(pddl.find("(at robot1 loc2)") != std::string::npos);
}

TEST_F(ProblemTest, CompleteProblempddl) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc1", "location"));
  problem.add_object(Object("loc2", "location"));
  problem.add_fact(Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(Predicate("connected", {"loc1", "loc2"}));
  problem.add_goal(Predicate("at", {"robot1", "loc2"}));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(define (problem") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:domain easy_plan_domain)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:objects") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:init") != std::string::npos);
  EXPECT_TRUE(pddl.find("(:goal") != std::string::npos);
  EXPECT_TRUE(pddl.find("(and") != std::string::npos);
}

TEST_F(ProblemTest, MultipleGoals) {
  Problem problem;

  problem.add_object(Object("robot1", "robot"));
  problem.add_object(Object("loc2", "location"));
  problem.add_object(Object("item1", "item"));
  problem.add_goal(Predicate("at", {"robot1", "loc2"}));
  problem.add_goal(Predicate("holding", {"robot1", "item1"}));

  std::string pddl = problem.to_pddl();

  EXPECT_TRUE(pddl.find("(and") != std::string::npos);
  EXPECT_TRUE(pddl.find("(at robot1 loc2)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(holding robot1 item1)") != std::string::npos);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
