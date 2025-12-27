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

#include "rclcpp/rclcpp.hpp"

#include "easy_plan_knowledge_base/knowledge_base_node.hpp"

#include "easy_plan_msgs/srv/add_fact.hpp"
#include "easy_plan_msgs/srv/add_goal.hpp"
#include "easy_plan_msgs/srv/add_object.hpp"
#include "easy_plan_msgs/srv/add_predicate.hpp"
#include "easy_plan_msgs/srv/add_type.hpp"
#include "easy_plan_msgs/srv/clear_knowledge_base.hpp"
#include "easy_plan_msgs/srv/get_facts.hpp"
#include "easy_plan_msgs/srv/get_goals.hpp"
#include "easy_plan_msgs/srv/get_objects.hpp"
#include "easy_plan_msgs/srv/get_predicates.hpp"
#include "easy_plan_msgs/srv/get_types.hpp"
#include "easy_plan_msgs/srv/remove_fact.hpp"
#include "easy_plan_msgs/srv/remove_goal.hpp"
#include "easy_plan_msgs/srv/remove_object.hpp"
#include "easy_plan_msgs/srv/remove_predicate.hpp"
#include "easy_plan_msgs/srv/remove_type.hpp"

using namespace easy_plan_knowledge_base;

class KnowledgeBaseNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<KnowledgeBaseNode>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Start executor in a separate thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Create a client node for testing
    client_node_ = rclcpp::Node::make_shared("test_client_node");
  }

  void TearDown() override {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(const std::string &service_name) {
    auto client = client_node_->create_client<ServiceT>(service_name);
    EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    return client;
  }

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr
  call_service(typename rclcpp::Client<ServiceT>::SharedPtr client,
               typename ServiceT::Request::SharedPtr request) {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node_, future,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      return future.get();
    }
    return nullptr;
  }

  std::shared_ptr<KnowledgeBaseNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Node::SharedPtr client_node_;
};

// ==================== Type Service Tests ====================

TEST_F(KnowledgeBaseNodeTest, AddTypeService) {
  auto client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");

  auto request = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  request->type = "robot";

  auto response = call_service<easy_plan_msgs::srv::AddType>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
}

TEST_F(KnowledgeBaseNodeTest, GetTypesService) {
  // First add a type
  auto add_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto add_request = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  add_request->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(add_client, add_request);

  // Then get types
  auto client = create_client<easy_plan_msgs::srv::GetTypes>(
      "/knowledge_base_node/get_types");
  auto request = std::make_shared<easy_plan_msgs::srv::GetTypes::Request>();

  auto response = call_service<easy_plan_msgs::srv::GetTypes>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->types.size(), 1u);
  EXPECT_EQ(response->types[0], "robot");
}

TEST_F(KnowledgeBaseNodeTest, RemoveTypeService) {
  // First add a type
  auto add_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto add_request = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  add_request->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(add_client, add_request);

  // Then remove it
  auto client = create_client<easy_plan_msgs::srv::RemoveType>(
      "/knowledge_base_node/remove_type");
  auto request = std::make_shared<easy_plan_msgs::srv::RemoveType::Request>();
  request->type = "robot";

  auto response =
      call_service<easy_plan_msgs::srv::RemoveType>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
}

// ==================== Object Service Tests ====================

TEST_F(KnowledgeBaseNodeTest, AddObjectService) {
  // First add the type
  auto type_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto type_request = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_request->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_request);

  // Now add the object
  auto client = create_client<easy_plan_msgs::srv::AddObject>(
      "/knowledge_base_node/add_object");

  auto request = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  request->object.name = "robot1";
  request->object.type = "robot";

  auto response = call_service<easy_plan_msgs::srv::AddObject>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
}

TEST_F(KnowledgeBaseNodeTest, GetObjectsService) {
  // First add the type
  auto type_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto type_request = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_request->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_request);

  // Then add an object
  auto add_client = create_client<easy_plan_msgs::srv::AddObject>(
      "/knowledge_base_node/add_object");
  auto add_request =
      std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  add_request->object.name = "robot1";
  add_request->object.type = "robot";
  call_service<easy_plan_msgs::srv::AddObject>(add_client, add_request);

  // Then get objects
  auto client = create_client<easy_plan_msgs::srv::GetObjects>(
      "/knowledge_base_node/get_objects");
  auto request = std::make_shared<easy_plan_msgs::srv::GetObjects::Request>();

  auto response =
      call_service<easy_plan_msgs::srv::GetObjects>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->objects.size(), 1u);
  EXPECT_EQ(response->objects[0].name, "robot1");
}

// ==================== Fact Service Tests ====================

TEST_F(KnowledgeBaseNodeTest, AddFactService) {
  // First add types
  auto type_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto type_req1 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req1->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req1);
  auto type_req2 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req2->type = "location";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req2);

  // Add objects
  auto obj_client = create_client<easy_plan_msgs::srv::AddObject>(
      "/knowledge_base_node/add_object");
  auto obj_req1 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req1->object.name = "robot1";
  obj_req1->object.type = "robot";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req1);
  auto obj_req2 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req2->object.name = "loc1";
  obj_req2->object.type = "location";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req2);

  // Add predicate types
  auto pred_type_req1 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req1->type = "?r";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req1);
  auto pred_type_req2 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req2->type = "?l";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req2);

  // Add predicate definition
  auto pred_client = create_client<easy_plan_msgs::srv::AddPredicate>(
      "/knowledge_base_node/add_predicate");
  auto pred_req =
      std::make_shared<easy_plan_msgs::srv::AddPredicate::Request>();
  pred_req->predicate.name = "at";
  pred_req->predicate.arguments = {"?r", "?l"};
  call_service<easy_plan_msgs::srv::AddPredicate>(pred_client, pred_req);

  // Now add fact
  auto client = create_client<easy_plan_msgs::srv::AddFact>(
      "/knowledge_base_node/add_fact");

  auto request = std::make_shared<easy_plan_msgs::srv::AddFact::Request>();
  request->fact.name = "at";
  request->fact.arguments = {"robot1", "loc1"};

  auto response = call_service<easy_plan_msgs::srv::AddFact>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
}

TEST_F(KnowledgeBaseNodeTest, GetFactsService) {
  // First add types
  auto type_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto type_req1 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req1->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req1);
  auto type_req2 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req2->type = "location";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req2);

  // Add objects
  auto obj_client = create_client<easy_plan_msgs::srv::AddObject>(
      "/knowledge_base_node/add_object");
  auto obj_req1 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req1->object.name = "robot1";
  obj_req1->object.type = "robot";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req1);
  auto obj_req2 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req2->object.name = "loc1";
  obj_req2->object.type = "location";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req2);

  // Add predicate types
  auto pred_type_req1 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req1->type = "?r";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req1);
  auto pred_type_req2 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req2->type = "?l";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req2);

  // Add predicate definition
  auto pred_client = create_client<easy_plan_msgs::srv::AddPredicate>(
      "/knowledge_base_node/add_predicate");
  auto pred_req =
      std::make_shared<easy_plan_msgs::srv::AddPredicate::Request>();
  pred_req->predicate.name = "at";
  pred_req->predicate.arguments = {"?r", "?l"};
  call_service<easy_plan_msgs::srv::AddPredicate>(pred_client, pred_req);

  // Then add a fact
  auto add_client = create_client<easy_plan_msgs::srv::AddFact>(
      "/knowledge_base_node/add_fact");
  auto add_request = std::make_shared<easy_plan_msgs::srv::AddFact::Request>();
  add_request->fact.name = "at";
  add_request->fact.arguments = {"robot1", "loc1"};
  call_service<easy_plan_msgs::srv::AddFact>(add_client, add_request);

  // Then get facts
  auto client = create_client<easy_plan_msgs::srv::GetFacts>(
      "/knowledge_base_node/get_facts");
  auto request = std::make_shared<easy_plan_msgs::srv::GetFacts::Request>();

  auto response = call_service<easy_plan_msgs::srv::GetFacts>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->facts.size(), 1u);
  EXPECT_EQ(response->facts[0].name, "at");
}

// ==================== Goal Service Tests ====================

TEST_F(KnowledgeBaseNodeTest, AddGoalService) {
  // First add types
  auto type_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto type_req1 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req1->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req1);
  auto type_req2 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req2->type = "location";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req2);

  // Add objects
  auto obj_client = create_client<easy_plan_msgs::srv::AddObject>(
      "/knowledge_base_node/add_object");
  auto obj_req1 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req1->object.name = "robot1";
  obj_req1->object.type = "robot";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req1);
  auto obj_req2 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req2->object.name = "loc2";
  obj_req2->object.type = "location";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req2);

  // Add predicate types
  auto pred_type_req1 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req1->type = "?r";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req1);
  auto pred_type_req2 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req2->type = "?l";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req2);

  // Add predicate definition
  auto pred_client = create_client<easy_plan_msgs::srv::AddPredicate>(
      "/knowledge_base_node/add_predicate");
  auto pred_req =
      std::make_shared<easy_plan_msgs::srv::AddPredicate::Request>();
  pred_req->predicate.name = "at";
  pred_req->predicate.arguments = {"?r", "?l"};
  call_service<easy_plan_msgs::srv::AddPredicate>(pred_client, pred_req);

  // Now add goal
  auto client = create_client<easy_plan_msgs::srv::AddGoal>(
      "/knowledge_base_node/add_goal");

  auto request = std::make_shared<easy_plan_msgs::srv::AddGoal::Request>();
  request->goal.name = "at";
  request->goal.arguments = {"robot1", "loc2"};

  auto response = call_service<easy_plan_msgs::srv::AddGoal>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
}

TEST_F(KnowledgeBaseNodeTest, GetGoalsService) {
  // First add types
  auto type_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto type_req1 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req1->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req1);
  auto type_req2 = std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  type_req2->type = "location";
  call_service<easy_plan_msgs::srv::AddType>(type_client, type_req2);

  // Add objects
  auto obj_client = create_client<easy_plan_msgs::srv::AddObject>(
      "/knowledge_base_node/add_object");
  auto obj_req1 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req1->object.name = "robot1";
  obj_req1->object.type = "robot";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req1);
  auto obj_req2 = std::make_shared<easy_plan_msgs::srv::AddObject::Request>();
  obj_req2->object.name = "loc2";
  obj_req2->object.type = "location";
  call_service<easy_plan_msgs::srv::AddObject>(obj_client, obj_req2);

  // Add predicate types
  auto pred_type_req1 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req1->type = "?r";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req1);
  auto pred_type_req2 =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  pred_type_req2->type = "?l";
  call_service<easy_plan_msgs::srv::AddType>(type_client, pred_type_req2);

  // Add predicate definition
  auto pred_client = create_client<easy_plan_msgs::srv::AddPredicate>(
      "/knowledge_base_node/add_predicate");
  auto pred_req =
      std::make_shared<easy_plan_msgs::srv::AddPredicate::Request>();
  pred_req->predicate.name = "at";
  pred_req->predicate.arguments = {"?r", "?l"};
  call_service<easy_plan_msgs::srv::AddPredicate>(pred_client, pred_req);

  // Then add a goal
  auto add_client = create_client<easy_plan_msgs::srv::AddGoal>(
      "/knowledge_base_node/add_goal");
  auto add_request = std::make_shared<easy_plan_msgs::srv::AddGoal::Request>();
  add_request->goal.name = "at";
  add_request->goal.arguments = {"robot1", "loc2"};
  call_service<easy_plan_msgs::srv::AddGoal>(add_client, add_request);

  // Then get goals
  auto client = create_client<easy_plan_msgs::srv::GetGoals>(
      "/knowledge_base_node/get_goals");
  auto request = std::make_shared<easy_plan_msgs::srv::GetGoals::Request>();

  auto response = call_service<easy_plan_msgs::srv::GetGoals>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->goals.size(), 1u);
  EXPECT_EQ(response->goals[0].name, "at");
}

// ==================== Clear Service Tests ====================

TEST_F(KnowledgeBaseNodeTest, ClearAllService) {
  // Add some data
  auto add_type_client = create_client<easy_plan_msgs::srv::AddType>(
      "/knowledge_base_node/add_type");
  auto add_type_request =
      std::make_shared<easy_plan_msgs::srv::AddType::Request>();
  add_type_request->type = "robot";
  call_service<easy_plan_msgs::srv::AddType>(add_type_client, add_type_request);

  auto add_fact_client = create_client<easy_plan_msgs::srv::AddFact>(
      "/knowledge_base_node/add_fact");
  auto add_fact_request =
      std::make_shared<easy_plan_msgs::srv::AddFact::Request>();
  add_fact_request->fact.name = "at";
  add_fact_request->fact.arguments = {"robot1", "loc1"};
  call_service<easy_plan_msgs::srv::AddFact>(add_fact_client, add_fact_request);

  // Clear all
  auto client = create_client<easy_plan_msgs::srv::ClearKnowledgeBase>(
      "/knowledge_base_node/clear");
  auto request =
      std::make_shared<easy_plan_msgs::srv::ClearKnowledgeBase::Request>();

  auto response =
      call_service<easy_plan_msgs::srv::ClearKnowledgeBase>(client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);

  // Verify data is cleared
  auto get_types_client = create_client<easy_plan_msgs::srv::GetTypes>(
      "/knowledge_base_node/get_types");
  auto get_types_request =
      std::make_shared<easy_plan_msgs::srv::GetTypes::Request>();
  auto get_types_response = call_service<easy_plan_msgs::srv::GetTypes>(
      get_types_client, get_types_request);
  EXPECT_TRUE(get_types_response->types.empty());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
