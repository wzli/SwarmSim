#include <gtest/gtest.h>
#include <swarm_sim/map_gen.hpp>
#include <swarm_sim/bin_router.hpp>

using namespace swarm_sim;

bool save_map(const MapGen& map_gen, const char* file) {
    auto fp = fopen(file, "w");
    if (!fp) {
        return false;
    }
    fprintf(fp, "type, id, x, y, z, t\r\n");
    int id = 0;
    for (auto& ele : map_gen.elevators) {
        fprintf(fp, "%u, %u, %f, %f, %f, %d\r\n", 0, id++, ele->position.get<0>(),
                ele->position.get<1>(), ele->position.get<2>(), 0);
    }
    id = 0;
    for (auto& bin : map_gen.bins) {
        fprintf(fp, "%u, %u, %f, %f, %f, %d\r\n", 1, id++, bin->position.get<0>(),
                bin->position.get<1>(), bin->position.get<2>(), 0);
    }
    id = 0;
    for (auto& bot : map_gen.bots) {
        fprintf(fp, "%u, %u, %f, %f, %f, %d\r\n", 2, id++, bot->position.get<0>(),
                bot->position.get<1>(), bot->position.get<2>(), 0);
    }

    BinRouter router;
    BinRouter::Config config{
            PathSearch::Config{""},
            FLT_MAX,
            FLT_MAX,
            10,
            50000,
            500,
            false,
    };
    std::vector<Nodes> dst_vec;
    dst_vec.reserve(map_gen.bins.size());
    for (auto& src : map_gen.bins) {
        dst_vec.emplace_back(Nodes{src});
    }

    auto dst_node = map_gen.graph.findNode({1, 1, 1});
    for (int i = 0; i < 3; ++i) {
        dst_vec[i] = {map_gen.graph.findNode({i + 1, i + 1, i})};
        assert(dst_vec[i][0]);
    }
    assert(!router.generateBinPaths(config, map_gen.bins, dst_vec, fp));
    return !fclose(fp);
}

TEST(map_gen, generate) {
    MapGen map_gen(MapGen::Config{10,  // rows
            10,                        // cols
            3,                         // floors
            200,                       // n_bins
            10,                        // n_bots,
            // elevators
            {{0, 0}, {0, 9}, {9, 0}, {9, 9}}});
    save_map(map_gen, "map.csv");
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
