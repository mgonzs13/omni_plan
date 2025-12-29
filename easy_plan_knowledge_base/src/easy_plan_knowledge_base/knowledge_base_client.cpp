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

#include "easy_plan_knowledge_base/knowledge_base_client.hpp"

using namespace easy_plan_knowledge_base;
using namespace std::chrono_literals;

KnowledgeBaseClient::KnowledgeBaseClient(const std::string &node_name) {
  // Create node
  this->node_ = rclcpp::Node::make_shared(node_name);

  // Create service clients - Types
  this->get_types_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetTypes>("get_types");
  this->add_type_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddType>("add_type");
  this->add_types_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddTypes>("add_types");
  this->remove_type_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveType>(
          "remove_type");
  this->remove_types_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveTypes>(
          "remove_types");

  // Create service clients - Objects
  this->get_objects_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetObjects>(
          "get_objects");
  this->add_object_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddObject>("add_object");
  this->add_objects_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddObjects>(
          "add_objects");
  this->remove_object_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveObject>(
          "remove_object");
  this->remove_objects_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveObjects>(
          "remove_objects");

  // Create service clients - Predicates
  this->get_predicates_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetPredicates>(
          "get_predicates");
  this->add_predicate_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddPredicate>(
          "add_predicate");
  this->add_predicates_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddPredicates>(
          "add_predicates");
  this->remove_predicate_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemovePredicate>(
          "remove_predicate");
  this->remove_predicates_client_ =
      node_->create_client<easy_plan_msgs::srv::RemovePredicates>(
          "remove_predicates");

  // Create service clients - Facts
  this->get_facts_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetFacts>("get_facts");
  this->add_fact_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddFact>("add_fact");
  this->add_facts_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddFacts>("add_facts");
  this->remove_fact_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveFact>(
          "remove_fact");
  this->remove_facts_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveFacts>(
          "remove_facts");

  // Create service clients - Goals
  this->get_goals_client_ =
      this->node_->create_client<easy_plan_msgs::srv::GetGoals>("get_goals");
  this->add_goal_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddGoal>("add_goal");
  this->add_goals_client_ =
      this->node_->create_client<easy_plan_msgs::srv::AddGoals>("add_goals");
  this->remove_goal_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveGoal>(
          "remove_goal");
  this->remove_goals_client_ =
      this->node_->create_client<easy_plan_msgs::srv::RemoveGoals>(
          "remove_goals");

  // Create service client - Clear
  this->clear_client_ =
      this->node_->create_client<easy_plan_msgs::srv::ClearKnowledgeBase>(
          "clear");

  // Subscribe to knowledge updates
  this->knowledge_update_sub_ =
      this->node_->create_subscription<easy_plan_msgs::msg::KnowledgeUpdate>(
          "knowledge_updates", 10,
          std::bind(&KnowledgeBaseClient::knowledge_update_callback, this,
                    std::placeholders::_1));

  // Create executor and start spinning in separate thread
  this->executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor_->add_node(this->node_);
  this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });
}

KnowledgeBaseClient::~KnowledgeBaseClient() {
  if (this->executor_) {
    this->executor_->cancel();
  }
  if (this->executor_thread_.joinable()) {
    this->executor_thread_.join();
  }
}

// ==================== Type Operations ====================
bool KnowledgeBaseClient::add_type(const std::string &type) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  request->type = type;

  this->add_type_client_->wait_for_service();
  auto future = this->add_type_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_types(const std::vector<std::string> &types) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddTypes::Request>();
  request->types = types;

  this->add_types_client_->wait_for_service();
  auto future = this->add_types_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_type(const std::string &type) {
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveType::Request>();
  request->type = type;

  this->remove_type_client_->wait_for_service();
  auto future = this->remove_type_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_types(const std::vector<std::string> &types) {
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveTypes::Request>();
  request->types = types;

  this->remove_types_client_->wait_for_service();
  auto future = this->remove_types_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<std::string> KnowledgeBaseClient::get_types() {
  auto request = std::make_shared<easy_plan_msgs::srv::GetTypes::Request>();

  this->get_types_client_->wait_for_service();
  auto future = this->get_types_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->types;
  }
  return {};
}

// ==================== Object Operations ====================
bool KnowledgeBaseClient::add_object(const std::string &name,
                                     const std::string &type) {
  return this->add_object(easy_plan::pddl::Object(name, type));
}

bool KnowledgeBaseClient::add_object(const easy_plan::pddl::Object &object) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  request->object = this->object_to_msg(object);

  this->add_object_client_->wait_for_service();
  auto future = this->add_object_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_objects(
    const std::vector<easy_plan::pddl::Object> &objects) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddObjects::Request>();
  for (const auto &obj : objects) {
    request->objects.push_back(this->object_to_msg(obj));
  }

  this->add_objects_client_->wait_for_service(1s);
  auto future = this->add_objects_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_object(const std::string &name,
                                        const std::string &type) {
  return this->remove_object(easy_plan::pddl::Object(name, type));
}

bool KnowledgeBaseClient::remove_object(const easy_plan::pddl::Object &object) {
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveObject::Request>();
  request->object = this->object_to_msg(object);

  this->remove_object_client_->wait_for_service();
  auto future = this->remove_object_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_objects(
    const std::vector<easy_plan::pddl::Object> &objects) {
  auto request =
      std::make_shared<easy_plan_msgs::srv::RemoveObjects::Request>();
  for (const auto &obj : objects) {
    request->objects.push_back(this->object_to_msg(obj));
  }

  this->remove_objects_client_->wait_for_service();
  auto future = this->remove_objects_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<easy_plan::pddl::Object> KnowledgeBaseClient::get_objects() {
  auto request = std::make_shared<easy_plan_msgs::srv::GetObjects::Request>();

  this->get_objects_client_->wait_for_service();
  auto future = this->get_objects_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    auto response = future.get();
    std::vector<easy_plan::pddl::Object> objects;
    for (const auto &obj_msg : response->objects) {
      objects.push_back(this->msg_to_object(obj_msg));
    }
    return objects;
  }
  return {};
}

// ==================== Predicate Operations ====================
bool KnowledgeBaseClient::add_predicate(const std::string &name,
                                        const std::vector<std::string> &args) {
  return this->add_predicate(easy_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::add_predicate(
    const easy_plan::pddl::Predicate &predicate) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddPredicate::Request>();
  request->predicate = this->predicate_to_msg(predicate);

  this->add_predicate_client_->wait_for_service();
  auto future = this->add_predicate_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_predicates(
    const std::vector<easy_plan::pddl::Predicate> &predicates) {
  auto request =
      std::make_shared<easy_plan_msgs::srv::AddPredicates::Request>();
  for (const auto &pred : predicates) {
    request->predicates.push_back(this->predicate_to_msg(pred));
  }

  this->add_predicates_client_->wait_for_service();
  auto future = this->add_predicates_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_predicate(
    const std::string &name, const std::vector<std::string> &args) {
  return this->remove_predicate(easy_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::remove_predicate(
    const easy_plan::pddl::Predicate &predicate) {
  auto request =
      std::make_shared<easy_plan_msgs::srv::RemovePredicate::Request>();
  request->predicate = this->predicate_to_msg(predicate);

  this->remove_predicate_client_->wait_for_service();
  auto future = this->remove_predicate_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_predicates(
    const std::vector<easy_plan::pddl::Predicate> &predicates) {
  auto request =
      std::make_shared<easy_plan_msgs::srv::RemovePredicates::Request>();
  for (const auto &pred : predicates) {
    request->predicates.push_back(this->predicate_to_msg(pred));
  }

  this->remove_predicates_client_->wait_for_service();
  auto future = this->remove_predicates_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<easy_plan::pddl::Predicate> KnowledgeBaseClient::get_predicates() {
  auto request =
      std::make_shared<easy_plan_msgs::srv::GetPredicates::Request>();

  this->get_predicates_client_->wait_for_service();
  auto future = this->get_predicates_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    auto response = future.get();
    std::vector<easy_plan::pddl::Predicate> predicates;
    for (const auto &pred_msg : response->predicates) {
      predicates.push_back(this->msg_to_predicate(pred_msg));
    }
    return predicates;
  }
  return {};
}

// ==================== Fact Operations ====================
bool KnowledgeBaseClient::add_fact(const std::string &name,
                                   const std::vector<std::string> &args) {
  return this->add_fact(easy_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::add_fact(const easy_plan::pddl::Predicate &fact) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddFact::Request>();
  request->fact = this->predicate_to_msg(fact);

  this->add_fact_client_->wait_for_service();
  auto future = this->add_fact_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_facts(
    const std::vector<easy_plan::pddl::Predicate> &facts) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddFacts::Request>();
  for (const auto &fact : facts) {
    request->facts.push_back(this->predicate_to_msg(fact));
  }

  this->add_facts_client_->wait_for_service();
  auto future = this->add_facts_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_fact(const std::string &name,
                                      const std::vector<std::string> &args) {
  return this->remove_fact(easy_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::remove_fact(const easy_plan::pddl::Predicate &fact) {
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveFact::Request>();
  request->fact = this->predicate_to_msg(fact);

  this->remove_fact_client_->wait_for_service();
  auto future = this->remove_fact_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_facts(
    const std::vector<easy_plan::pddl::Predicate> &facts) {
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveFacts::Request>();
  for (const auto &fact : facts) {
    request->facts.push_back(this->predicate_to_msg(fact));
  }

  this->remove_facts_client_->wait_for_service();
  auto future = this->remove_facts_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<easy_plan::pddl::Predicate>
KnowledgeBaseClient::get_facts(const std::string &name) {
  auto request = std::make_shared<easy_plan_msgs::srv::GetFacts::Request>();
  request->name = name;

  this->get_facts_client_->wait_for_service();
  auto future = this->get_facts_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    auto response = future.get();
    std::vector<easy_plan::pddl::Predicate> facts;
    for (const auto &fact_msg : response->facts) {
      facts.push_back(this->msg_to_predicate(fact_msg));
    }
    return facts;
  }
  return {};
}

// ==================== Goal Operations ====================
bool KnowledgeBaseClient::add_goal(const std::string &name,
                                   const std::vector<std::string> &args) {
  return this->add_goal(easy_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::add_goal(const easy_plan::pddl::Predicate &goal) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddGoal::Request>();
  request->goal = this->predicate_to_msg(goal);

  this->add_goal_client_->wait_for_service();
  auto future = this->add_goal_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_goals(
    const std::vector<easy_plan::pddl::Predicate> &goals) {
  auto request = std::make_shared<easy_plan_msgs::srv::AddGoals::Request>();
  for (const auto &goal : goals) {
    request->goals.push_back(this->predicate_to_msg(goal));
  }

  this->add_goals_client_->wait_for_service();
  auto future = this->add_goals_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_goal(const std::string &name,
                                      const std::vector<std::string> &args) {
  return this->remove_goal(easy_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::remove_goal(const easy_plan::pddl::Predicate &goal) {
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveGoal::Request>();
  request->goal = this->predicate_to_msg(goal);

  this->remove_goal_client_->wait_for_service();
  auto future = this->remove_goal_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_goals(
    const std::vector<easy_plan::pddl::Predicate> &goals) {
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveGoals::Request>();
  for (const auto &goal : goals) {
    request->goals.push_back(this->predicate_to_msg(goal));
  }

  this->remove_goals_client_->wait_for_service();
  auto future = this->remove_goals_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<easy_plan::pddl::Predicate> KnowledgeBaseClient::get_goals() {
  auto request = std::make_shared<easy_plan_msgs::srv::GetGoals::Request>();

  this->get_goals_client_->wait_for_service();
  auto future = this->get_goals_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    auto response = future.get();
    std::vector<easy_plan::pddl::Predicate> goals;
    for (const auto &goal_msg : response->goals) {
      goals.push_back(this->msg_to_predicate(goal_msg));
    }
    return goals;
  }
  return {};
}

bool KnowledgeBaseClient::has_goals() {
  auto goals = this->get_goals();
  return !goals.empty();
}

// ==================== Clear ====================
bool KnowledgeBaseClient::clear() {
  auto request =
      std::make_shared<easy_plan_msgs::srv::ClearKnowledgeBase::Request>();

  this->clear_client_->wait_for_service();
  auto future = this->clear_client_->async_send_request(request);

  if (future.wait_for(5s) == std::future_status::ready) {
    return future.get()->success;
  }

  return false;
}

// ==================== Callback Management ====================
void KnowledgeBaseClient::add_knowledge_update_callback(
    KnowledgeUpdateCallback callback) {
  this->callbacks_.push_back(callback);
}

void KnowledgeBaseClient::knowledge_update_callback(
    const easy_plan_msgs::msg::KnowledgeUpdate::SharedPtr msg) {
  for (const auto &callback : this->callbacks_) {
    callback(msg);
  }
}

// ==================== Helper Methods ====================
easy_plan_msgs::msg::Object KnowledgeBaseClient::object_to_msg(
    const easy_plan::pddl::Object &object) const {
  easy_plan_msgs::msg::Object msg;
  msg.name = object.get_name();
  msg.type = object.get_type();
  return msg;
}

easy_plan::pddl::Object KnowledgeBaseClient::msg_to_object(
    const easy_plan_msgs::msg::Object &msg) const {
  return easy_plan::pddl::Object(msg.name, msg.type);
}

easy_plan_msgs::msg::Predicate KnowledgeBaseClient::predicate_to_msg(
    const easy_plan::pddl::Predicate &predicate) const {
  easy_plan_msgs::msg::Predicate msg;
  msg.name = predicate.get_name();
  msg.arguments = predicate.get_args();
  msg.negated = predicate.is_negated();
  msg.time = easy_plan_msgs::msg::Predicate::AT_END;
  return msg;
}

easy_plan::pddl::Predicate KnowledgeBaseClient::msg_to_predicate(
    const easy_plan_msgs::msg::Predicate &msg) const {
  return easy_plan::pddl::Predicate(msg.name, msg.arguments, msg.negated);
}
