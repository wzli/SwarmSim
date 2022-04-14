#pragma once
#include <decentralized_path_auction/graph.hpp>

#include <algorithm>
#include <random>
#include <vector>

namespace swarm_sim {

using namespace decentralized_path_auction;

struct MapGen {
    struct Config {
        size_t rows;
        size_t cols;
        size_t floors;
        size_t n_bins;
        size_t n_bots;
        std::vector<std::pair<size_t, size_t>> elevators;
    };

    MapGen(const Config& config);

    Graph graph;
    std::vector<NodePtr> bins;
    std::vector<NodePtr> bots;
};

}  // namespace swarm_sim
