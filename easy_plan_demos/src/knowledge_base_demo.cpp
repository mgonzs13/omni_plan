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

#include "easy_plan_msgs/msg/object.hpp"
#include "easy_plan_msgs/msg/predicate.hpp"
#include "easy_plan_msgs/srv/add_fact.hpp"
#include "easy_plan_msgs/srv/add_goal.hpp"
#include "easy_plan_msgs/srv/add_object.hpp"
#include "easy_plan_msgs/srv/add_predicate.hpp"
#include "easy_plan_msgs/srv/add_type.hpp"
#include "easy_plan_msgs/srv/get_facts.hpp"
#include "easy_plan_msgs/srv/get_goals.hpp"
#include "easy_plan_msgs/srv/get_objects.hpp"

using namespace std::chrono_literals;

class KnowledgeBaseDemoNode : public rclcpp::Node {
public:
  KnowledgeBaseDemoNode() : Node("knowledge_base_demo") {
    // Create service clients
    this->add_type_client_ =
        this->create_client<easy_plan_msgs::srv::AddType>("add_type");
    this->add_object_client_ =
        this->create_client<easy_plan_msgs::srv::AddObject>("add_object");
    this->add_predicate_client_ =
        this->create_client<easy_plan_msgs::srv::AddPredicate>("add_predicate");
    this->add_fact_client_ =
        this->create_client<easy_plan_msgs::srv::AddFact>("add_fact");
    this->add_goal_client_ =
        this->create_client<easy_plan_msgs::srv::AddGoal>("add_goal");
    this->get_objects_client_ =
        this->create_client<easy_plan_msgs::srv::GetObjects>("get_objects");
    this->get_facts_client_ =
        this->create_client<easy_plan_msgs::srv::GetFacts>("get_facts");
    this->get_goals_client_ =
        this->create_client<easy_plan_msgs::srv::GetGoals>("get_goals");

    // Wait for services to be available
    RCLCPP_INFO(this->get_logger(), "Waiting for knowledge base services...");
    this->add_type_client_->wait_for_service();
    this->add_object_client_->wait_for_service();
    this->add_predicate_client_->wait_for_service();
    this->add_fact_client_->wait_for_service();
    this->add_goal_client_->wait_for_service();

    // Populate knowledge base
    this->populate_knowledge_base();

    // Display knowledge base contents
    this->display_knowledge_base();
  }

private:
  void populate_knowledge_base() {
    RCLCPP_INFO(this->get_logger(), "Populating knowledge base...");

    // Add types
    this->add_type("robot");
    this->add_type("room");

    // Add objects - robot
    this->add_object("leia", "robot");

    // Add objects - rooms
    this->add_object("entrance", "room");
    this->add_object("kitchen", "room");
    this->add_object("bedroom", "room");
    this->add_object("dinning", "room");
    this->add_object("bathroom", "room");
    this->add_object("chargingroom", "room");

    // Add predicates
    this->add_predicate("connected", {"room", "room"});
    this->add_predicate("charging_point_at", {"room"});
    this->add_predicate("battery_low", {"robot"});
    this->add_predicate("battery_full", {"robot"});
    this->add_predicate("robot_at", {"robot", "room"});

    // Add facts - connected predicates (bidirectional)
    this->add_fact("connected", {"entrance", "dinning"});
    this->add_fact("connected", {"dinning", "entrance"});

    this->add_fact("connected", {"dinning", "kitchen"});
    this->add_fact("connected", {"kitchen", "dinning"});

    this->add_fact("connected", {"dinning", "bedroom"});
    this->add_fact("connected", {"bedroom", "dinning"});

    this->add_fact("connected", {"bathroom", "bedroom"});
    this->add_fact("connected", {"bedroom", "bathroom"});

    this->add_fact("connected", {"chargingroom", "kitchen"});
    this->add_fact("connected", {"kitchen", "chargingroom"});

    // Add facts - other predicates
    this->add_fact("charging_point_at", {"chargingroom"});
    this->add_fact("battery_low", {"leia"});
    this->add_fact("robot_at", {"leia", "entrance"});

    // Add goal
    this->add_goal("robot_at", {"leia", "bathroom"});

    RCLCPP_INFO(this->get_logger(), "Knowledge base populated successfully!");
  }

  void add_type(const std::string &type_name) {
    auto request = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
    request->type = type_name;

    auto future = add_type_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        RCLCPP_DEBUG(this->get_logger(), "Added type: %s", type_name.c_str());
      }
    }
  }

  void add_object(const std::string &name, const std::string &type) {
    auto request = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
    request->object.name = name;
    request->object.type = type;

    auto future = this->add_object_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        RCLCPP_DEBUG(this->get_logger(), "Added object: %s (%s)", name.c_str(),
                     type.c_str());
      }
    }
  }

  void add_predicate(const std::string &name,
                     const std::vector<std::string> &args) {
    auto request =
        std::make_shared<easy_plan_msgs::srv::AddPredicate::Request>();
    request->predicate.name = name;
    request->predicate.arguments = args;

    auto future = this->add_predicate_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        RCLCPP_DEBUG(this->get_logger(), "Added predicate: %s", name.c_str());
      }
    }
  }

  void add_fact(const std::string &name, const std::vector<std::string> &args) {
    auto request = std::make_shared<easy_plan_msgs::srv::AddFact::Request>();
    request->fact.name = name;
    request->fact.arguments = args;

    auto future = this->add_fact_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        RCLCPP_DEBUG(this->get_logger(), "Added fact: %s", name.c_str());
      }
    }
  }

  void add_goal(const std::string &name, const std::vector<std::string> &args) {
    auto request = std::make_shared<easy_plan_msgs::srv::AddGoal::Request>();
    request->goal.name = name;
    request->goal.arguments = args;

    auto future = this->add_goal_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        RCLCPP_DEBUG(this->get_logger(), "Added goal: %s", name.c_str());
      }
    }
  }

  void display_knowledge_base() {
    std::cout << "\nKnowledge Base Demo" << std::endl;
    std::cout << "===================" << std::endl;

    // Get and display objects
    auto objects_request =
        std::make_shared<easy_plan_msgs::srv::GetObjects::Request>();
    auto objects_future =
        this->get_objects_client_->async_send_request(objects_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           objects_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto objects_response = objects_future.get();
      std::cout << "Objects (" << objects_response->objects.size()
                << "):" << std::endl;
      for (const auto &obj : objects_response->objects) {
        std::cout << "  " << obj.name << " - " << obj.type << std::endl;
      }
    }

    // Get and display facts
    auto facts_request =
        std::make_shared<easy_plan_msgs::srv::GetFacts::Request>();
    facts_request->name = ""; // Get all facts
    auto facts_future =
        this->get_facts_client_->async_send_request(facts_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           facts_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto facts_response = facts_future.get();
      std::cout << "Facts (" << facts_response->facts.size()
                << "):" << std::endl;
      for (const auto &fact : facts_response->facts) {
        std::cout << "  (" << fact.name;
        for (const auto &arg : fact.arguments) {
          std::cout << " " << arg;
        }
        std::cout << ")" << std::endl;
      }
    }

    // Get and display goals
    auto goals_request =
        std::make_shared<easy_plan_msgs::srv::GetGoals::Request>();
    auto goals_future =
        this->get_goals_client_->async_send_request(goals_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           goals_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto goals_response = goals_future.get();
      std::cout << "Goals (" << goals_response->goals.size()
                << "):" << std::endl;
      for (const auto &goal : goals_response->goals) {
        std::cout << "  (" << goal.name;
        for (const auto &arg : goal.arguments) {
          std::cout << " " << arg;
        }
        std::cout << ")" << std::endl;
      }
    }
  }

  rclcpp::Client<easy_plan_msgs::srv::AddType>::SharedPtr add_type_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddObject>::SharedPtr add_object_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddPredicate>::SharedPtr
      add_predicate_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddFact>::SharedPtr add_fact_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddGoal>::SharedPtr add_goal_client_;
  rclcpp::Client<easy_plan_msgs::srv::GetObjects>::SharedPtr
      get_objects_client_;
  rclcpp::Client<easy_plan_msgs::srv::GetFacts>::SharedPtr get_facts_client_;
  rclcpp::Client<easy_plan_msgs::srv::GetGoals>::SharedPtr get_goals_client_;
};

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create and run the knowledge base demo node
  auto node = std::make_shared<KnowledgeBaseDemoNode>();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
