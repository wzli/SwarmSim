#include <gtest/gtest.h>
#include <swarm_sim/map_gen.hpp>

using namespace swarm_sim;

bool save_map(const MapGen& map_gen, const char* file) {
    auto fp = fopen(file, "w");
    if (!fp) {
        return false;
    }
    fprintf(fp, "type, id, x, y, z, t\r\n");
    int id = 0;
    for (auto& ele : map_gen.elevators) {
        fprintf(fp, "%u, %u, %f, %f, %f\r\n", 0, id++, ele->position.get<0>(),
                ele->position.get<1>(), ele->position.get<2>());
    }
    id = 0;
    for (auto& bin : map_gen.bins) {
        fprintf(fp, "%u, %u, %f, %f, %f\r\n", 1, id++, bin->position.get<0>(),
                bin->position.get<1>(), bin->position.get<2>());
    }
    id = 0;
    for (auto& bot : map_gen.bots) {
        fprintf(fp, "%u, %u, %f, %f, %f\r\n", 2, id++, bot->position.get<0>(),
                bot->position.get<1>(), bot->position.get<2>());
    }
    return !fclose(fp);
}

TEST(map_gen, generate) {
    MapGen map_gen(MapGen::Config{10,  // rows
            10,                        // cols
            3,                         // floors
            50,                        // n_bins
            10,                        // n_bots,
            // elevators
            {{0, 0}, {0, 9}, {9, 0}, {9, 9}}});
    save_map(map_gen, "map.csv");
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
