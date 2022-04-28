#include <swarm_sim/bin_router.hpp>
#include <algorithm>

namespace swarm_sim {

BinRouter::BinRouter(Config config)
        : _config(std::move(config))
        , _map(_config.map_gen_config) {}

BinRouter::Error BinRouter::solve(const std::vector<BinRequest>& requests, const char* save_file) {
    // create and initialize bin destination vector
    std::vector<Nodes> dst_vec;
    dst_vec.reserve(_map.bins.size());
    for (auto& src : _map.bins) {
        dst_vec.emplace_back(Nodes{src});
    }

    // add bin requests to destination vector
    for (auto& req : requests) {
        if (req.bin_id >= _map.bins.size()) {
            return REQUEST_BIN_ID_OUT_OF_RANGE;
        }
        auto dst_node = _map.graph.findNode({static_cast<float>(req.col),
                static_cast<float>(req.row), static_cast<float>(req.floor)});
        if (!dst_node) {
            return REQUEST_BIN_NODE_NOT_FOUND;
        }
        if (dst_node->state >= Node::NO_PARKING) {
            return REQUEST_BIN_NODE_NOT_PARKABLE;
        }
        dst_vec[req.bin_id] = {dst_node};
    }

    // open save file
    auto fp = fopen(save_file, "w");
    if (!fp) {
        return FILE_OPEN_FAIL;
    }

    // write static marker entries to data file
    fprintf(fp, "stage, type, id, x, y, z, t\r\n");

    // generate bin routes
    if (Error error = generateBinPaths(dst_vec)) {
        fclose(fp);
        return error;
    }

    int stage = 0;
    saveEntities(fp, stage);
    savePaths(_bin_path_planner.getPathSync(), fp, stage++, false);

    // generate traversal order of bin routes
    std::vector<int> order;
    generateTraversalOrder(order, _bin_path_planner.getPathSync());

    // generate bin paths by processing one chunk of the traversal at a time
    for (auto cur = order.cbegin(); cur != order.cend();) {
        auto bin_idx = cur;
        saveEntities(fp, stage);
        if (Error error = generateRobotPaths(cur, order.cend())) {
            fclose(fp);
            return error;
        }
        for (; bin_idx != cur; ++bin_idx) {
            auto& bin_path =
                    _bin_path_planner.getPathSync().getPaths().at(std::to_string(*bin_idx)).path;
            savePath(*bin_idx + _map.bots.size(), bin_path, fp, stage, false);
        }
        savePaths(_robot_path_planner.getPathSync(), fp, stage++, true);
    }

    fclose(fp);
    return SUCCESS;
}

BinRouter::Error BinRouter::generateRobotPaths(std::vector<int>::const_iterator& order_cur,
        const std::vector<int>::const_iterator order_end) {
    // create new empty graph from config
    MapGen::Config map_config = _config.map_gen_config;
    map_config.n_bins = 0;
    map_config.n_bots = 0;
    MapGen robot_map(map_config);

    // create path search config
    PathSearch::Config path_search_config;
    path_search_config.travel_time = [this](const NodePtr& prev, const NodePtr& cur,
                                             const NodePtr& next) {
        return customTravelTime(prev, cur, next);
    };

    // build destination candidates vector
    Nodes dst_candidates;
    std::unordered_map<NodePtr, std::pair<int, NodePtr>> dst_map;
    for (const auto order_begin = order_cur;
            order_cur != order_begin + _map.bots.size() && order_cur != order_end; ++order_cur) {
        printf("%d, ", *order_cur);
        auto& bin_path =
                _bin_path_planner.getPathSync().getPaths().at(std::to_string(*order_cur)).path;
        auto bin_position = bin_path.front().node->position;
        auto bin_node = robot_map.graph.findNode(bin_position);
        assert(bin_node);
        dst_candidates.emplace_back(bin_node);
        dst_map.emplace(bin_node, std::pair<int, NodePtr>{*order_cur, bin_path.back().node});
    }
    puts("end");

    // build robot path requests
    _path_requests.clear();
    for (size_t i = 0; i < _map.bots.size(); ++i) {
        NodePtr robot_loc = robot_map.graph.findNode(_map.bots[i]->position);
        assert(robot_loc);
        path_search_config.agent_id = std::to_string(i);
        float fallback_cost = _config.fallback_cost;
        // need to lower fallback costs and increase price increment when there are less
        // destinations than robots otherwise they keep competing until out of iterations
        if (dst_candidates.size() < _map.bots.size()) {
            fallback_cost /= 5;
            path_search_config.price_increment *= 10;
        }
        MultiPathPlanner::Request request{dst_candidates, FLT_MAX, path_search_config,
                {{robot_loc}, _config.iterations, fallback_cost}};
        _path_requests.emplace_back(std::move(request));
    }

    // plan robot routes
    _robot_path_planner.plan(_config.planner_config, _path_requests);

    auto& path_sync = _robot_path_planner.getPathSync();
    auto& results = _robot_path_planner.getResults();
    for (size_t i = 0; i < results.size(); ++i) {
        // skip bins that don't move
        auto& path = path_sync.getPaths().at(std::to_string(i)).path;
        printf("robot id %ld search %d sync %d length %ld\n", i, results[i].search_error,
                results[i].sync_error, path.size());
        // return GENERATE_ROBOT_PATHS_FAIL;
        if (results[i].search_error > PathSearch::FALLBACK_DIVERTED || results[i].sync_error) {
            return GENERATE_ROBOT_PATHS_FAIL;
        }
        // move bin and robot to destination of bin
        if (results[i].search_error == PathSearch::SUCCESS) {
            auto [bin_id, bin_dst_node] = dst_map.at(path.back().node);
            _map.bots[i] = bin_dst_node;
            _map.bins[bin_id] = bin_dst_node;
        } else if (results[i].search_error == PathSearch::FALLBACK_DIVERTED) {
            auto dst_node = _map.graph.findNode(path.back().node->position);
            assert(dst_node);
            _map.bots[i] = dst_node;
        }
    }
    return SUCCESS;
}

BinRouter::Error BinRouter::generateBinPaths(const std::vector<Nodes>& dst_vec) {
    auto& src_vec = _map.bins;
    assert(src_vec.size() == dst_vec.size());
    _path_requests.clear();
    // create path search config
    PathSearch::Config path_search_config;
    path_search_config.travel_time = [this](const NodePtr& prev, const NodePtr& cur,
                                             const NodePtr& next) {
        return customTravelTime(prev, cur, next);
    };

    // create requests vector
    for (size_t i = 0; i < src_vec.size(); ++i) {
        auto& src = src_vec[i];
        auto& dst = dst_vec[i];
        float fallback_cost = dst.size() == 1 && dst.front() == src ? _config.blocking_fallback_cost
                                                                    : _config.fallback_cost;
        path_search_config.agent_id = std::to_string(i);
        MultiPathPlanner::Request request{
                dst, FLT_MAX, path_search_config, {{src}, _config.iterations, fallback_cost}};
        _path_requests.emplace_back(std::move(request));
    }
    // plan routes
    _bin_path_planner.plan(_config.planner_config, _path_requests);
    auto& path_sync = _bin_path_planner.getPathSync();
    auto& results = _bin_path_planner.getResults();
    for (size_t i = 0; i < results.size(); ++i) {
        // skip bins that don't move
        size_t len = path_sync.getPaths().at(std::to_string(i)).path.size();
        if ((dst_vec[i].empty() || dst_vec[i].front() == src_vec[i]) && len < 2) {
            continue;
        }
        printf("id %ld search %d sync %d length %ld\n", i, results[i].search_error,
                results[i].sync_error, len);
        if (results[i].search_error > PathSearch::FALLBACK_DIVERTED || results[i].sync_error) {
            return GENERATE_BIN_PATHS_FAIL;
        }
    }
    return SUCCESS;
}

void BinRouter::saveEntities(FILE* save_file, int stage) {
    assert(save_file);
    int id = 0;
    for (auto& ele : _map.elevators) {
        fprintf(save_file, "%d, %u, %u, %f, %f, %f, %d\r\n", stage, ELEVATOR, id++,
                ele->position.get<0>(), ele->position.get<1>(), ele->position.get<2>(), 0);
    }
    id = 0;
    for (auto& bin : _map.bins) {
        fprintf(save_file, "%d, %u, %u, %f, %f, %f, %d\r\n", stage, BIN, id++,
                bin->position.get<0>(), bin->position.get<1>(), bin->position.get<2>(), 0);
    }
    id = 0;
    for (auto& bot : _map.bots) {
        fprintf(save_file, "%d, %u, %u, %f, %f, %f, %d\r\n", stage, ROBOT, id++,
                bot->position.get<0>(), bot->position.get<1>(), bot->position.get<2>(), 0);
    }
}

void BinRouter::savePath(int id, const Path& path, FILE* save_file, int stage, bool under) {
    for (auto visit = path.begin(); visit != path.end(); ++visit) {
        auto& bids = visit->node->auction.getBids();
        // lambda to save a csv entry
        auto save_entry = [&](int z_offset) {
            float z = std::distance(bids.find(visit->price), bids.end()) - 1;
            fprintf(save_file, "%d, %u, %d, %f, %f, %f, %f\r\n", stage, PATH, id,
                    visit->node->position.get<0>(), visit->node->position.get<1>(),
                    (visit + z_offset)->node->position.get<2>(), under ? -0.25f - z : 0.25f + z);
        };
        // if the node is an elevator
        if (visit->node->custom_data) {
            // save entry if there is a previous floor
            if (visit > path.begin()) {
                save_entry(-1);
            }
            // save entry if there is a next floor
            if (visit < path.end()) {
                save_entry(1);
            }
        } else {
            // otherwise save entry for current floor
            save_entry(0);
        }
    }
}

void BinRouter::savePaths(const PathSync& path_sync, FILE* save_file, int stage, bool under) {
    assert(save_file);
    for (auto& [id, info] : path_sync.getPaths()) {
        if (info.path.size() < 2) {
            continue;
        }
        savePath(std::stoi(id), info.path, save_file, stage, under);
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
            // printf("done %d\n", id);
        }
        // increment visit count
        if (visit_count[id] < 255) {
            ++visit_count[id];
        }
    }

    for (int id : traversal_order) {
        printf("->%d", id);
    }
    puts("");
}

float BinRouter::customTravelTime(const NodePtr& prev, const NodePtr& cur, const NodePtr& next) {
    // initialize to 2D manhattan distance
    // prev only exists for adjacent queries
    float t = prev ? 1.0f
                   : std::abs(cur->position.get<0>() - next->position.get<0>()) +
                              std::abs(cur->position.get<1>() - next->position.get<1>());
    // add elevator duration if exiting elevator or floor changed
    if (cur->custom_data ||
            !(next->custom_data || cur->position.get<2>() == next->position.get<2>())) {
        t += _config.elevator_duration;
    }
    return t;
}

}  // namespace swarm_sim
