// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#include "gtest/gtest.h"
#include "src/behavior_uct_single_agent.hpp"
#include "modules/commons/params/default_params.hpp"

#define UNIT_TESTING
#define DEBUG

#include <cstdio>

TEST(test_single_agent_mcts, temp )
{


    modules::commons::DefaultParams params;
    modules::models::behavior::BehaviorUCTSingleAgent behavior(&params);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}