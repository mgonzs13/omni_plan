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
#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan_knowledge_base/knowledge_base_client.hpp"

using namespace omni_plan_knowledge_base;
using namespace std::chrono_literals;

TEST(KnowledgeBaseClientTimeoutTest, AddTypeReturnsFalseWithoutServer) {
  auto client = std::make_shared<KnowledgeBaseClient>("timeout_test_client",
                                                      "timeout_test_namespace");

  auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client->add_type("robot"));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 30s);
}

TEST(KnowledgeBaseClientTimeoutTest, GetTypesReturnsEmptyWithoutServer) {
  auto client = std::make_shared<KnowledgeBaseClient>("timeout_test_client2",
                                                      "timeout_test_namespace");

  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(client->get_types().empty());
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 30s);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
