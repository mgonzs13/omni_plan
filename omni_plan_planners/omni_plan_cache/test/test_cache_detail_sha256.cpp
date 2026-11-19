// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <gtest/gtest.h>

#include "omni_plan_cache/cache_planner.hpp"
#include "omni_plan_cache/detail/sha256.hpp"

using omni_plan_cache::detail::Sha256;
using omni_plan_cache::detail::sha256_hex;

TEST(Sha256Test, KnownVector) {
  EXPECT_EQ(sha256_hex("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(Sha256Test, EmptyVector) {
  EXPECT_EQ(sha256_hex(""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256Test, StreamingMatchesOneShot) {
  Sha256 h;
  h.update("hello ");
  h.update("world");
  EXPECT_EQ(h.final_hex(), sha256_hex("hello world"));
}

TEST(Sha256Test, CachePlannerHelperMatches) {
  EXPECT_EQ(omni_plan_cache::CachePlanner::sha256("hello"),
            sha256_hex("hello"));
}
