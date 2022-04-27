#include <gtest/gtest.h>
#include <swarm_sim/map_gen.hpp>
#include <swarm_sim/bin_router.hpp>

using namespace swarm_sim;

TEST(map_gen, generate) {
    BinRouter::Config config;
    config.elevator_duration = 10.0f;
    // path planner config
    config.fallback_cost = FLT_MAX;
    config.blocking_fallback_cost = 10.0f;
    config.iterations = 100000;
    // multi path planner config
    config.planner_config.rounds = 200;
    config.planner_config.n_threads = 8;
    config.planner_config.allow_indefinite_block = false;
    // map gen config
    config.map_gen_config.rows = 10;
    config.map_gen_config.cols = 10;
    config.map_gen_config.floors = 3;
    config.map_gen_config.n_bins = 100;
    config.map_gen_config.n_bots = 2;
    config.map_gen_config.elevators = {{0, 0}, {0, 9}, {9, 0}, {9, 9}};
    // generate solution
    BinRouter bin_router(std::move(config));
    ASSERT_EQ(BinRouter::SUCCESS, bin_router.solve(
                                          {
                                                  {0, 4, 0, 1},
                                                  {1, 5, 0, 1},
                                                  {2, 6, 0, 1},
                                          },
                                          "bin_routes.csv"));
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
