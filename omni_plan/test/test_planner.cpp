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
#include <vector>

#include "omni_plan/planner.hpp"

using namespace omni_plan;

/**
 * @brief Concrete planner for testing time parsing methods.
 */
class TestPlanner : public Planner {
public:
  TestPlanner() : Planner() {}

  std::string generate_plan(const std::string /*domain_path*/,
                            const std::string /*problem_path*/) const override {
    return "";
  }

  bool has_solution(const std::string & /*plan_str*/) const override {
    return false;
  }

  // Expose protected methods for testing
  float test_parse_start_time(const std::string &line) const {
    return parse_start_time(line);
  }

  float test_parse_duration(const std::string &line) const {
    return parse_duration(line);
  }

  std::pair<std::string, std::vector<std::string>>
  test_parse_action_line(const std::string &line) const {
    return parse_action_line(line);
  }

  std::vector<std::string>
  test_get_lines_with_actions(const std::string &plan_str) const {
    return get_lines_with_actions(plan_str);
  }
};

// ==================== Start Time Parsing Tests ====================
class PlannerParseTest : public ::testing::Test {
protected:
  TestPlanner planner_;
};

TEST_F(PlannerParseTest, ParseStartTimePopfFormat) {
  // POPF output format: "0.000: (move robot1 room1 room2) [10.000]"
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(
                      "0.000: (move robot1 room1 room2) [10.000]"),
                  0.0f);
}

TEST_F(PlannerParseTest, ParseStartTimeNonZero) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(
                      "10.500: (pick robot1 item1 room2) [5.000]"),
                  10.5f);
}

TEST_F(PlannerParseTest, ParseStartTimeLargeValue) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(
                      "123.456: (drop robot1 item1 room3) [2.500]"),
                  123.456f);
}

TEST_F(PlannerParseTest, ParseStartTimeNoColon) {
  EXPECT_FLOAT_EQ(
      planner_.test_parse_start_time("(move robot1 room1 room2) [10.000]"),
      0.0f);
}

TEST_F(PlannerParseTest, ParseStartTimeEmptyLine) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(""), 0.0f);
}

TEST_F(PlannerParseTest, ParseStartTimeInvalidNumber) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time("abc: (move) [10.0]"), 0.0f);
}

// ==================== Duration Parsing Tests ====================
TEST_F(PlannerParseTest, ParseDurationPopfFormat) {
  EXPECT_FLOAT_EQ(
      planner_.test_parse_duration("0.000: (move robot1 room1 room2) [10.000]"),
      10.0f);
}

TEST_F(PlannerParseTest, ParseDurationFractional) {
  EXPECT_FLOAT_EQ(
      planner_.test_parse_duration("0.000: (pick robot1 item1 room2) [5.750]"),
      5.75f);
}

TEST_F(PlannerParseTest, ParseDurationNoBrackets) {
  EXPECT_FLOAT_EQ(
      planner_.test_parse_duration("0.000: (move robot1 room1 room2)"), 0.0f);
}

TEST_F(PlannerParseTest, ParseDurationEmptyLine) {
  EXPECT_FLOAT_EQ(planner_.test_parse_duration(""), 0.0f);
}

TEST_F(PlannerParseTest, ParseDurationInvalidNumber) {
  EXPECT_FLOAT_EQ(planner_.test_parse_duration("0.000: (move robot1) [abc]"),
                  0.0f);
}

// ==================== Action Line Parsing Tests ====================
TEST_F(PlannerParseTest, ParseActionLinePopfFormat) {
  auto [name, params] = planner_.test_parse_action_line(
      "0.000: (move robot1 room1 room2) [10.000]");
  EXPECT_EQ(name, "move");
  ASSERT_EQ(params.size(), 3u);
  EXPECT_EQ(params[0], "robot1");
  EXPECT_EQ(params[1], "room1");
  EXPECT_EQ(params[2], "room2");
}

TEST_F(PlannerParseTest, ParseActionLineNoParams) {
  auto [name, params] =
      planner_.test_parse_action_line("0.000: (wait) [5.000]");
  EXPECT_EQ(name, "wait");
  EXPECT_EQ(params.size(), 0u);
}

// ==================== Lines Filtering Tests ====================
TEST_F(PlannerParseTest, GetLinesWithActionsMultiple) {
  std::string plan_str = "Some header line\n"
                         "0.000: (move robot1 room1 room2) [10.000]\n"
                         "0.000: (pick robot2 item2 room2) [5.000]\n"
                         "10.001: (pick robot1 item1 room2) [5.000]\n"
                         "Some footer line\n";

  auto lines = planner_.test_get_lines_with_actions(plan_str);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_TRUE(lines[0].find("move") != std::string::npos);
  EXPECT_TRUE(lines[1].find("robot2") != std::string::npos);
  EXPECT_TRUE(lines[2].find("10.001") != std::string::npos);
}

TEST_F(PlannerParseTest, GetLinesWithActionsEmpty) {
  auto lines = planner_.test_get_lines_with_actions("");
  EXPECT_EQ(lines.size(), 0u);
}

// ==================== Combined Time + Action Parsing ====================
TEST_F(PlannerParseTest, FullLineParsing) {
  std::string line = "15.250: (drop robot1 item1 room3) [3.500]";

  auto [name, params] = planner_.test_parse_action_line(line);
  float start_time = planner_.test_parse_start_time(line);
  float duration = planner_.test_parse_duration(line);

  EXPECT_EQ(name, "drop");
  ASSERT_EQ(params.size(), 3u);
  EXPECT_EQ(params[0], "robot1");
  EXPECT_EQ(params[1], "item1");
  EXPECT_EQ(params[2], "room3");
  EXPECT_FLOAT_EQ(start_time, 15.25f);
  EXPECT_FLOAT_EQ(duration, 3.5f);
}
