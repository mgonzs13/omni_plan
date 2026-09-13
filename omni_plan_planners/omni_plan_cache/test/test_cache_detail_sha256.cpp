// Copyright (C) 2026 Miguel Ángel González Santamarta
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
