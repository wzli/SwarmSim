#include <swarm_sim/bin_router.hpp>
#include <algorithm>

namespace swarm_sim {

BinRouter::Error BinRouter::generateBinPaths(const Config& config, const Nodes& src_vec,
        const std::vector<Nodes>& dst_vec, FILE* save_file) {
    assert(src_vec.size() == dst_vec.size());
    _requests.clear();
    // create requests vector
    for (size_t i = 0; i < src_vec.size(); ++i) {
        auto& src = src_vec[i];
        auto& dst = dst_vec[i];
        auto path_search_config = config.path_search_config;
        path_search_config.agent_id = std::to_string(i);
        MultiPathPlanner::Request request{dst, config.duration, std::move(path_search_config),
                {{src}, config.iterations, config.fallback_cost}};
        _requests.emplace_back(std::move(request));
    }
    // plan routes
    auto result = _multi_path_planner.plan(_requests, config.rounds, config.allow_block);
    if (result.search_error || result.sync_error) {
        printf("!!!!!!!!!ERROR path %d sync %d\r\n", result.search_error, result.sync_error);
        return FAILURE;
    }
    if (save_file) {
        savePaths(_multi_path_planner.getPathSync(), save_file);
    }
    return SUCCESS;
}

void BinRouter::savePaths(const PathSync& path_sync, FILE* save_file) {
    assert(save_file);
    for (auto& [id, info] : path_sync.getPaths()) {
        if (info.path.size() == 1) {
            continue;
        }
        for (auto& visit : info.path) {
            auto& bids = visit.node->auction.getBids();
            fprintf(save_file, "%u, %d, %f, %f, %f, %ld\r\n", PATH, std::stoi(id),
                    visit.node->position.get<0>(), visit.node->position.get<1>(),
                    visit.node->position.get<2>(),
                    std::distance(bids.find(visit.price), bids.end()) - 1);
        }
    }
}

}  // namespace swarm_sim
