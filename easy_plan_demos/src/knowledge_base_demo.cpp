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

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "easy_plan_knowledge_base/knowledge_base_client.hpp"

using namespace easy_plan_knowledge_base;

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a knowledge base client
  std::shared_ptr<KnowledgeBaseClient> kb_client =
      std::make_shared<KnowledgeBaseClient>("knowledge_base_demo");

  std::cout << "Populating knowledge base..." << std::endl;

  // Add types
  kb_client->add_type("robot");
  kb_client->add_type("room");

  // Add objects - robot
  kb_client->add_object("leia", "robot");

  // Add objects - rooms
  kb_client->add_object("entrance", "room");
  kb_client->add_object("kitchen", "room");
  kb_client->add_object("bedroom", "room");
  kb_client->add_object("dinning", "room");
  kb_client->add_object("bathroom", "room");
  kb_client->add_object("chargingroom", "room");

  // Add predicates
  kb_client->add_predicate("connected", {"room", "room"});
  kb_client->add_predicate("charging_point_at", {"room"});
  kb_client->add_predicate("battery_low", {"robot"});
  kb_client->add_predicate("battery_full", {"robot"});
  kb_client->add_predicate("robot_at", {"robot", "room"});

  // Add facts - connected predicates (bidirectional)
  kb_client->add_fact("connected", {"entrance", "dinning"});
  kb_client->add_fact("connected", {"dinning", "entrance"});

  kb_client->add_fact("connected", {"dinning", "kitchen"});
  kb_client->add_fact("connected", {"kitchen", "dinning"});

  kb_client->add_fact("connected", {"dinning", "bedroom"});
  kb_client->add_fact("connected", {"bedroom", "dinning"});

  kb_client->add_fact("connected", {"bathroom", "bedroom"});
  kb_client->add_fact("connected", {"bedroom", "bathroom"});

  kb_client->add_fact("connected", {"chargingroom", "kitchen"});
  kb_client->add_fact("connected", {"kitchen", "chargingroom"});

  // Add facts - other predicates
  kb_client->add_fact("charging_point_at", {"chargingroom"});
  kb_client->add_fact("battery_low", {"leia"});
  kb_client->add_fact("robot_at", {"leia", "entrance"});

  // Add goal
  kb_client->add_goal("robot_at", {"leia", "bathroom"});

  // Display knowledge base data
  std::cout << "Knowledge base populated successfully!" << std::endl;

  std::cout << "\nKnowledge Base Demo" << std::endl;
  std::cout << "===================" << std::endl;

  // Get and display objects
  auto objects = kb_client->get_objects();
  std::cout << "Objects (" << objects.size() << "):" << std::endl;
  for (const auto &obj : objects) {
    std::cout << "  " << obj.get_name() << " - " << obj.get_type() << std::endl;
  }

  // Get and display facts
  auto facts = kb_client->get_facts();
  std::cout << "Facts (" << facts.size() << "):" << std::endl;
  for (const auto &fact : facts) {
    std::cout << "  (" << fact.get_name();
    for (const auto &arg : fact.get_args()) {
      std::cout << " " << arg;
    }
    std::cout << ")" << std::endl;
  }

  // Get and display goals
  auto goals = kb_client->get_goals();
  std::cout << "Goals (" << goals.size() << "):" << std::endl;
  for (const auto &goal : goals) {
    std::cout << "  (" << goal.get_name();
    for (const auto &arg : goal.get_args()) {
      std::cout << " " << arg;
    }
    std::cout << ")" << std::endl;
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
