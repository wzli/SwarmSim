#pragma once

#include <swarm_sim/path_planner.hpp>
#include <string>
#include <unordered_map>

namespace swarm_sim {
using namespace decentralized_path_auction;

class BinRouter {
    enum Error {
        SUCCESS,
        FAILURE,
    };

    enum DataEntryType {
        ELEVATOR,
        BIN,
        ROBOT,
        PATH,
    };

public:
    struct Config {
        PathSearch::Config path_search_config;
        float duration;
        float fallback_cost;
        float blocking_fallback_cost;
        size_t iterations;
        size_t rounds;
        bool allow_block;
    };

    Error generateBinPaths(const Config& config, const Nodes& src_vec,
            const std::vector<Nodes>& dst_vec, FILE* save_file = nullptr);

private:
    void savePaths(const PathSync& path_sync, FILE* save_file);

    MultiPathPlanner _multi_path_planner;
    std::vector<MultiPathPlanner::Request> _requests;
};

}  // namespace swarm_sim
