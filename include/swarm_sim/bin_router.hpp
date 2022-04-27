#pragma once

#include <swarm_sim/path_planner.hpp>
#include <swarm_sim/map_gen.hpp>
#include <string>
#include <unordered_map>

namespace swarm_sim {
using namespace decentralized_path_auction;

class BinRouter {
public:
    enum Error {
        SUCCESS,
        FILE_OPEN_FAIL,
        REQUEST_BIN_ID_OUT_OF_RANGE,
        REQUEST_BIN_NODE_NOT_FOUND,
        REQUEST_BIN_NODE_NOT_PARKABLE,
        GENERATE_BIN_PATHS_FAIL,
        FAILURE,
    };

    enum DataEntryType {
        ELEVATOR,
        BIN,
        ROBOT,
        PATH,
    };

    struct Config {
        float elevator_duration;
        float fallback_cost;
        float blocking_fallback_cost;
        size_t iterations;
        MultiPathPlanner::Config planner_config;
        MapGen::Config map_gen_config;
    };

    struct BinRequest {
        size_t bin_id;
        size_t col;
        size_t row;
        size_t floor;
    };

    BinRouter(Config config);

    Error solve(const std::vector<BinRequest>& requests, const char* save_file);

private:
    float customTravelTime(const NodePtr& prev, const NodePtr& cur, const NodePtr& next);

    Error generateBinPaths(const std::vector<Nodes>& dst_vec, FILE* save_file = nullptr);

    void generateTraversalOrder(std::vector<int>& traversal_order, const PathSync& path_sync);

    void savePaths(const PathSync& path_sync, FILE* save_file);

    MultiPathPlanner _multi_path_planner;
    std::vector<MultiPathPlanner::Request> _requests;

    Config _config;
    MapGen _map;
};

}  // namespace swarm_sim
