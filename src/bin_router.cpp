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
        float fallback_cost = dst.size() == 1 && dst.front() == src ? config.blocking_fallback_cost
                                                                    : config.fallback_cost;
        path_search_config.agent_id = std::to_string(i);
        MultiPathPlanner::Request request{dst, config.duration, std::move(path_search_config),
                {{src}, config.iterations, fallback_cost}};
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
    std::vector<int> order;
    generateTraversalOrder(order, _multi_path_planner.getPathSync());
    return SUCCESS;
}

void BinRouter::savePaths(const PathSync& path_sync, FILE* save_file) {
    assert(save_file);
    for (auto& [id, info] : path_sync.getPaths()) {
        if (info.path.size() < 2) {
            continue;
        }
        for (auto visit = info.path.begin(); visit != info.path.end(); ++visit) {
            auto& bids = visit->node->auction.getBids();
            // lambda to save a csv entry
            auto save_entry = [&](int z_offset) {
                fprintf(save_file, "%u, %d, %f, %f, %f, %ld\r\n", PATH, std::stoi(id),
                        visit->node->position.get<0>(), visit->node->position.get<1>(),
                        (visit + z_offset)->node->position.get<2>(),
                        std::distance(bids.find(visit->price), bids.end()) - 1);
            };
            // if the node is an elevator
            if (visit->node->custom_data) {
                // save entry if there is a previous floor
                if (visit > info.path.begin()) {
                    save_entry(-1);
                }
                // save entry if there is a next floor
                if (visit < info.path.end()) {
                    save_entry(1);
                }
            } else {
                // otherwise save entry for current floor
                save_entry(0);
            }
        }
    }
}

void BinRouter::generateTraversalOrder(
        std::vector<int>& traversal_order, const PathSync& path_sync) {
    // initialize visit_count lookup table
    auto& paths = path_sync.getPaths();
    std::vector<uint8_t> visit_count(paths.size());

    // initialize traversal stack in order of IDs
    std::vector<int> stack;
    stack.reserve(paths.size() * 2);
    for (int i = paths.size() - 1; i >= 0; --i) {
        stack.push_back(i);
    }

    // clear output buffer
    traversal_order.clear();
    traversal_order.reserve(paths.size());

    // traverse path dependencies
    while (!stack.empty()) {
        // get path from ID at back of stack
        int id = stack.back();
        auto& path = paths.at(std::to_string(id)).path;
        // remove if already visited
        if (visit_count[id] > 0) {
            stack.pop_back();
        } else if (path_sync.checkWaitStatus(std::to_string(id)).blocked_progress < path.size()) {
            // add dependencies on first visit
            printf("%d: ", id);
            for (auto visit = path.rbegin(); visit != path.rend(); ++visit) {
                auto& bids = visit->node->auction.getBids();
                auto higher_bid = visit->node->auction.getHigherBid(visit->price);
                if (higher_bid != bids.end()) {
                    int dep_id = std::stoi(higher_bid->second.bidder);
                    if (visit_count[dep_id] == 0) {
                        stack.push_back(dep_id);
                    }
                    printf("%d ", dep_id);
                }
            }
            puts("");
        }
        // if a path is revisited, it means all dependencies are met
        // log in traversal order but filter out trivial paths
        if (visit_count[id] == 1 && path.size() > 1) {
            traversal_order.push_back(id);
            printf("done %d\n", id);
        }
        // increment visit count
        if (visit_count[id] < 255) {
            ++visit_count[id];
        }
    }

    for (int id : traversal_order) {
        printf("traverse %d\n", id);
    }
    puts("");
}

}  // namespace swarm_sim
