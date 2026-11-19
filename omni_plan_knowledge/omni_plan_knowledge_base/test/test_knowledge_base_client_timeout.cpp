// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include <chrono>
#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_knowledge_base/knowledge_base_client.hpp"

using namespace omni_plan_knowledge_base;
using namespace std::chrono_literals;

TEST(KnowledgeBaseClientTimeoutTest, AddTypeReturnsFalseWithoutServer) {
  // Short timeout: this test only verifies the failure path, it must not
  // wait the production default (5 s) per call.
  auto client = std::make_shared<KnowledgeBaseClient>(
      "timeout_test_client", "timeout_test_namespace", 100ms);

  auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client->add_type("robot"));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 2s);
}

TEST(KnowledgeBaseClientTimeoutTest, GetTypesReturnsEmptyWithoutServer) {
  auto client = std::make_shared<KnowledgeBaseClient>(
      "timeout_test_client2", "timeout_test_namespace", 100ms);

  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(client->get_types().empty());
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 2s);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
