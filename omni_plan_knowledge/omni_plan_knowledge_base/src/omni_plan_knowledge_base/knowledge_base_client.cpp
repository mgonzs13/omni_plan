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

#include "omni_plan_knowledge_base/knowledge_base_client.hpp"

using namespace omni_plan_knowledge_base;
using namespace std::chrono_literals;

KnowledgeBaseClient::KnowledgeBaseClient(
    const std::string &node_name, const std::string &node_namespace,
    std::chrono::milliseconds service_timeout)
    : service_timeout_(service_timeout) {
  // Create node options
  rclcpp::NodeOptions node_options;
  node_options.use_global_arguments(false);

  // Create node
  this->node_ =
      rclcpp::Node::make_shared(node_name, node_namespace, node_options);

  // Create service clients - Types
  this->get_types_client_ =
      this->node_->create_client<omni_plan_msgs::srv::GetTypes>("get_types");
  this->add_type_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddType>("add_type");
  this->add_types_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddTypes>("add_types");
  this->remove_type_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveType>(
          "remove_type");
  this->remove_types_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveTypes>(
          "remove_types");

  // Create service clients - Objects
  this->get_objects_client_ =
      this->node_->create_client<omni_plan_msgs::srv::GetObjects>(
          "get_objects");
  this->add_object_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddObject>("add_object");
  this->add_objects_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddObjects>(
          "add_objects");
  this->remove_object_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveObject>(
          "remove_object");
  this->remove_objects_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveObjects>(
          "remove_objects");

  // Create service clients - Predicates
  this->get_predicates_client_ =
      this->node_->create_client<omni_plan_msgs::srv::GetPredicates>(
          "get_predicates");
  this->add_predicate_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddPredicate>(
          "add_predicate");
  this->add_predicates_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddPredicates>(
          "add_predicates");
  this->remove_predicate_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemovePredicate>(
          "remove_predicate");
  this->remove_predicates_client_ =
      node_->create_client<omni_plan_msgs::srv::RemovePredicates>(
          "remove_predicates");

  // Create service clients - Facts
  this->get_facts_client_ =
      this->node_->create_client<omni_plan_msgs::srv::GetFacts>("get_facts");
  this->add_fact_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddFact>("add_fact");
  this->add_facts_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddFacts>("add_facts");
  this->remove_fact_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveFact>(
          "remove_fact");
  this->remove_facts_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveFacts>(
          "remove_facts");

  // Create service clients - Goals
  this->get_goals_client_ =
      this->node_->create_client<omni_plan_msgs::srv::GetGoals>("get_goals");
  this->add_goal_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddGoal>("add_goal");
  this->add_goals_client_ =
      this->node_->create_client<omni_plan_msgs::srv::AddGoals>("add_goals");
  this->remove_goal_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveGoal>(
          "remove_goal");
  this->remove_goals_client_ =
      this->node_->create_client<omni_plan_msgs::srv::RemoveGoals>(
          "remove_goals");

  // Create service client - Clear
  this->clear_client_ =
      this->node_->create_client<omni_plan_msgs::srv::ClearKnowledgeBase>(
          "clear");

  // Subscribe to knowledge updates
  this->knowledge_update_sub_ =
      this->node_->create_subscription<omni_plan_msgs::msg::KnowledgeUpdate>(
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
  auto request = std::make_shared<omni_plan_msgs::srv::AddType::Request>();
  request->type = type;

  if (!this->add_type_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_type' not available.");
    return false;
  }

  auto future = this->add_type_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_types(const std::vector<std::string> &types) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddTypes::Request>();
  request->types = types;

  if (!this->add_types_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_types' not available.");
    return false;
  }

  auto future = this->add_types_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_type(const std::string &type) {
  auto request = std::make_shared<omni_plan_msgs::srv::RemoveType::Request>();
  request->type = type;

  if (!this->remove_type_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_type' not available.");
    return false;
  }

  auto future = this->remove_type_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_types(const std::vector<std::string> &types) {
  auto request = std::make_shared<omni_plan_msgs::srv::RemoveTypes::Request>();
  request->types = types;

  if (!this->remove_types_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_types' not available.");
    return false;
  }

  auto future = this->remove_types_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<std::string> KnowledgeBaseClient::get_types() {
  auto request = std::make_shared<omni_plan_msgs::srv::GetTypes::Request>();

  if (!this->get_types_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'get_types' not available.");
    return {};
  }

  auto future = this->get_types_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->types;
  }
  return {};
}

// ==================== Object Operations ====================
bool KnowledgeBaseClient::add_object(const std::string &name,
                                     const std::string &type) {
  return this->add_object(omni_plan::pddl::Object(name, type));
}

bool KnowledgeBaseClient::add_object(const omni_plan::pddl::Object &object) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddObject::Request>();
  request->object = this->object_to_msg(object);

  if (!this->add_object_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_object' not available.");
    return false;
  }

  auto future = this->add_object_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_objects(
    const std::vector<omni_plan::pddl::Object> &objects) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddObjects::Request>();
  for (const auto &obj : objects) {
    request->objects.push_back(this->object_to_msg(obj));
  }

  if (!this->add_objects_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_objects' not available.");
    return false;
  }

  auto future = this->add_objects_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_object(const std::string &name,
                                        const std::string &type) {
  return this->remove_object(omni_plan::pddl::Object(name, type));
}

bool KnowledgeBaseClient::remove_object(const omni_plan::pddl::Object &object) {
  auto request = std::make_shared<omni_plan_msgs::srv::RemoveObject::Request>();
  request->object = this->object_to_msg(object);

  if (!this->remove_object_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_object' not available.");
    return false;
  }

  auto future = this->remove_object_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_objects(
    const std::vector<omni_plan::pddl::Object> &objects) {
  auto request =
      std::make_shared<omni_plan_msgs::srv::RemoveObjects::Request>();
  for (const auto &obj : objects) {
    request->objects.push_back(this->object_to_msg(obj));
  }

  if (!this->remove_objects_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_objects' not available.");
    return false;
  }

  auto future = this->remove_objects_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<omni_plan::pddl::Object> KnowledgeBaseClient::get_objects() {
  auto request = std::make_shared<omni_plan_msgs::srv::GetObjects::Request>();

  if (!this->get_objects_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'get_objects' not available.");
    return {};
  }

  auto future = this->get_objects_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    auto response = future.get();
    std::vector<omni_plan::pddl::Object> objects;
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
  return this->add_predicate(omni_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::add_predicate(
    const omni_plan::pddl::Predicate &predicate) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddPredicate::Request>();
  request->predicate = this->predicate_to_msg(predicate);

  if (!this->add_predicate_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_predicate' not available.");
    return false;
  }

  auto future = this->add_predicate_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_predicates(
    const std::vector<omni_plan::pddl::Predicate> &predicates) {
  auto request =
      std::make_shared<omni_plan_msgs::srv::AddPredicates::Request>();
  for (const auto &pred : predicates) {
    request->predicates.push_back(this->predicate_to_msg(pred));
  }

  if (!this->add_predicates_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_predicates' not available.");
    return false;
  }

  auto future = this->add_predicates_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_predicate(
    const std::string &name, const std::vector<std::string> &args) {
  return this->remove_predicate(omni_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::remove_predicate(
    const omni_plan::pddl::Predicate &predicate) {
  auto request =
      std::make_shared<omni_plan_msgs::srv::RemovePredicate::Request>();
  request->predicate = this->predicate_to_msg(predicate);

  if (!this->remove_predicate_client_->wait_for_service(
          this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_predicate' not available.");
    return false;
  }

  auto future = this->remove_predicate_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_predicates(
    const std::vector<omni_plan::pddl::Predicate> &predicates) {
  auto request =
      std::make_shared<omni_plan_msgs::srv::RemovePredicates::Request>();
  for (const auto &pred : predicates) {
    request->predicates.push_back(this->predicate_to_msg(pred));
  }

  if (!this->remove_predicates_client_->wait_for_service(
          this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_predicates' not available.");
    return false;
  }

  auto future = this->remove_predicates_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<omni_plan::pddl::Predicate> KnowledgeBaseClient::get_predicates() {
  auto request =
      std::make_shared<omni_plan_msgs::srv::GetPredicates::Request>();

  if (!this->get_predicates_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'get_predicates' not available.");
    return {};
  }

  auto future = this->get_predicates_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    auto response = future.get();
    std::vector<omni_plan::pddl::Predicate> predicates;
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
  return this->add_fact(omni_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::add_fact(const omni_plan::pddl::Predicate &fact) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddFact::Request>();
  request->fact = this->predicate_to_msg(fact);

  if (!this->add_fact_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_fact' not available.");
    return false;
  }

  auto future = this->add_fact_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_facts(
    const std::vector<omni_plan::pddl::Predicate> &facts) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddFacts::Request>();
  for (const auto &fact : facts) {
    request->facts.push_back(this->predicate_to_msg(fact));
  }

  if (!this->add_facts_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_facts' not available.");
    return false;
  }

  auto future = this->add_facts_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_fact(const std::string &name,
                                      const std::vector<std::string> &args) {
  return this->remove_fact(omni_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::remove_fact(const omni_plan::pddl::Predicate &fact) {
  auto request = std::make_shared<omni_plan_msgs::srv::RemoveFact::Request>();
  request->fact = this->predicate_to_msg(fact);

  if (!this->remove_fact_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_fact' not available.");
    return false;
  }

  auto future = this->remove_fact_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_facts(
    const std::vector<omni_plan::pddl::Predicate> &facts) {
  auto request = std::make_shared<omni_plan_msgs::srv::RemoveFacts::Request>();
  for (const auto &fact : facts) {
    request->facts.push_back(this->predicate_to_msg(fact));
  }

  if (!this->remove_facts_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_facts' not available.");
    return false;
  }

  auto future = this->remove_facts_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<omni_plan::pddl::Predicate>
KnowledgeBaseClient::get_facts(const std::string &name) {
  auto request = std::make_shared<omni_plan_msgs::srv::GetFacts::Request>();
  request->name = name;

  if (!this->get_facts_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'get_facts' not available.");
    return {};
  }

  auto future = this->get_facts_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    auto response = future.get();
    std::vector<omni_plan::pddl::Predicate> facts;
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
  return this->add_goal(omni_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::add_goal(const omni_plan::pddl::Predicate &goal) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddGoal::Request>();
  request->goal = this->predicate_to_msg(goal);

  if (!this->add_goal_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_goal' not available.");
    return false;
  }

  auto future = this->add_goal_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::add_goals(
    const std::vector<omni_plan::pddl::Predicate> &goals) {
  auto request = std::make_shared<omni_plan_msgs::srv::AddGoals::Request>();
  for (const auto &goal : goals) {
    request->goals.push_back(this->predicate_to_msg(goal));
  }

  if (!this->add_goals_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'add_goals' not available.");
    return false;
  }

  auto future = this->add_goals_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_goal(const std::string &name,
                                      const std::vector<std::string> &args) {
  return this->remove_goal(omni_plan::pddl::Predicate(name, args));
}

bool KnowledgeBaseClient::remove_goal(const omni_plan::pddl::Predicate &goal) {
  auto request = std::make_shared<omni_plan_msgs::srv::RemoveGoal::Request>();
  request->goal = this->predicate_to_msg(goal);

  if (!this->remove_goal_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_goal' not available.");
    return false;
  }

  auto future = this->remove_goal_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

bool KnowledgeBaseClient::remove_goals(
    const std::vector<omni_plan::pddl::Predicate> &goals) {
  auto request = std::make_shared<omni_plan_msgs::srv::RemoveGoals::Request>();
  for (const auto &goal : goals) {
    request->goals.push_back(this->predicate_to_msg(goal));
  }

  if (!this->remove_goals_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'remove_goals' not available.");
    return false;
  }

  auto future = this->remove_goals_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }
  return false;
}

std::vector<omni_plan::pddl::Predicate> KnowledgeBaseClient::get_goals() {
  auto request = std::make_shared<omni_plan_msgs::srv::GetGoals::Request>();

  if (!this->get_goals_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(),
                 "Service 'get_goals' not available.");
    return {};
  }

  auto future = this->get_goals_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    auto response = future.get();
    std::vector<omni_plan::pddl::Predicate> goals;
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
      std::make_shared<omni_plan_msgs::srv::ClearKnowledgeBase::Request>();

  if (!this->clear_client_->wait_for_service(this->service_timeout_)) {
    RCLCPP_ERROR(this->node_->get_logger(), "Service 'clear' not available.");
    return false;
  }

  auto future = this->clear_client_->async_send_request(request);

  if (future.wait_for(this->service_timeout_) == std::future_status::ready) {
    return future.get()->success;
  }

  return false;
}

// ==================== Callback Management ====================
std::size_t KnowledgeBaseClient::add_knowledge_update_callback(
    KnowledgeUpdateCallback callback) {
  std::lock_guard<std::mutex> lock(this->callbacks_mutex_);
  std::size_t callback_id = this->next_callback_id_++;
  this->callbacks_[callback_id] = std::move(callback);
  return callback_id;
}

bool KnowledgeBaseClient::remove_knowledge_update_callback(
    std::size_t callback_id) {
  std::lock_guard<std::mutex> lock(this->callbacks_mutex_);
  return this->callbacks_.erase(callback_id) > 0;
}

void KnowledgeBaseClient::knowledge_update_callback(
    const omni_plan_msgs::msg::KnowledgeUpdate::SharedPtr msg) {
  std::vector<KnowledgeUpdateCallback> callbacks;

  {
    std::lock_guard<std::mutex> lock(this->callbacks_mutex_);
    callbacks.reserve(this->callbacks_.size());
    for (const auto &entry : this->callbacks_) {
      callbacks.push_back(entry.second);
    }
  }

  for (const auto &callback : callbacks) {
    callback(msg);
  }
}

// ==================== Conversion Helpers ====================
uint8_t
KnowledgeBaseClient::timing_type_to_msg_time(omni_plan::pddl::Type type) {
  switch (type) {
  case omni_plan::pddl::START:
    return omni_plan_msgs::msg::Predicate::AT_START;
  case omni_plan::pddl::OVER_ALL:
    return omni_plan_msgs::msg::Predicate::OVER_ALL;
  case omni_plan::pddl::END:
    return omni_plan_msgs::msg::Predicate::AT_END;
  }

  return omni_plan_msgs::msg::Predicate::AT_START;
}

omni_plan::pddl::Type
KnowledgeBaseClient::msg_time_to_timing_type(uint8_t time) {
  switch (time) {
  case omni_plan_msgs::msg::Predicate::AT_START:
    return omni_plan::pddl::START;
  case omni_plan_msgs::msg::Predicate::OVER_ALL:
    return omni_plan::pddl::OVER_ALL;
  case omni_plan_msgs::msg::Predicate::AT_END:
    return omni_plan::pddl::END;
  default:
    break;
  }

  return omni_plan::pddl::START;
}

// ==================== Helper Methods ====================
omni_plan_msgs::msg::Object KnowledgeBaseClient::object_to_msg(
    const omni_plan::pddl::Object &object) const {
  omni_plan_msgs::msg::Object msg;
  msg.name = object.get_name();
  msg.type = object.get_type();
  return msg;
}

omni_plan::pddl::Object KnowledgeBaseClient::msg_to_object(
    const omni_plan_msgs::msg::Object &msg) const {
  return omni_plan::pddl::Object(msg.name, msg.type);
}

omni_plan_msgs::msg::Predicate KnowledgeBaseClient::predicate_to_msg(
    const omni_plan::pddl::Predicate &predicate) const {
  omni_plan_msgs::msg::Predicate msg;
  msg.name = predicate.get_name();
  msg.arguments = predicate.get_args();
  msg.negated = predicate.is_negated();

  const auto *timing =
      dynamic_cast<const omni_plan::pddl::TimingPredicate *>(&predicate);
  msg.time =
      timing != nullptr
          ? KnowledgeBaseClient::timing_type_to_msg_time(timing->get_type())
          : omni_plan_msgs::msg::Predicate::AT_START;
  return msg;
}

omni_plan::pddl::Predicate KnowledgeBaseClient::msg_to_predicate(
    const omni_plan_msgs::msg::Predicate &msg) const {
  return omni_plan::pddl::Predicate(msg.name, msg.arguments, msg.negated);
}
