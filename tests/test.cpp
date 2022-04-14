#include <gtest/gtest.h>
#include <swarm_sim/map_gen.hpp>

using namespace swarm_sim;

bool save_graph(const Graph& graph, const char* file) {
    auto fp = fopen(file, "w");
    if (!fp) {
        return false;
    }
    fprintf(fp, "src_pos_x, src_pos_y, dst_pos_x, dst_pos_y\r\n");
    for (auto& [pos, node] : graph.getNodes()) {
        for (auto& adj_node : node->edges) {
            fprintf(fp, "%f, %f, %f, %f\r\n", pos.get<0>(), pos.get<1>(),
                    adj_node->position.get<0>(), adj_node->position.get<1>());
        }
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
    save_graph(map_gen.graph, "graph.csv");
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
