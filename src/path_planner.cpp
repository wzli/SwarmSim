#include <swarm_sim/path_planner.hpp>
#include <algorithm>

namespace decentralized_path_auction {

PathSearch::Error PathPlanner::plan(const PlanArgs& args, Nodes dst, float duration) {
    if (auto err = _path_search.setDestinations(std::move(dst), duration)) {
        return err;
    }
    _path = {_path_search.selectSource(args.src)};
    return _path_search.iterate(_path, args.iterations, args.fallback_cost);
}

PathSearch::Error PathPlanner::replan(const PlanArgs& args) {
    auto src_visit =
            args.src.size() > 1 ? _path_search.selectSource(args.src) : Visit{args.src.front()};
    // find if src node is in previous path
    auto found = std::find_if(_path.begin(), _path.end(),
            [&src_visit](auto visit) { return visit.node == src_visit.node; });
    // truncate previous path from the front up to src node
    _path.erase(_path.begin(), found);
    if (found == _path.end()) {
        _path.emplace_back(src_visit);
    }
    // run search with fallback
    auto err = _path_search.iterate(_path, args.iterations, args.fallback_cost);
    // reset and try again if path search failed
    if (err > PathSearch::FALLBACK_DIVERTED) {
        _path_search.resetCostEstimates();
        err = _path_search.iterate(_path, args.iterations, args.fallback_cost);
    }
    return err;
}

MultiPathPlanner::PlanResult MultiPathPlanner::plan(
        const std::vector<Request>& requests, size_t rounds, bool allow_block) {
    _path_sync.clearPaths();
    _path_planners.clear();
    // initialize path planners and set destination
    for (auto& req : requests) {
        _path_planners.emplace_back(req.config);
        PlanResult result{&req, _path_planners.back().plan(req.args, req.dst, req.duration)};
        if (result.search_error) {
            return result;
        }
    }
    size_t path_id = 0;
    while (--rounds > 0) {
        PlanResult result{&requests.front()};
        for (auto& planner : _path_planners) {
            // replan path
            result.search_error = planner.replan(result.request->args);
            if (result.search_error > PathSearch::ITERATIONS_REACHED) {
                return result;
            }
            // add path to path_sync
            result.sync_error =
                    _path_sync.updatePath(planner.getId(), planner.getPath(), path_id++);
            if (result.sync_error) {
                return result;
            }
            // terminate when all paths satisfactory
            if (std::all_of(_path_planners.begin(), _path_planners.end(), [&](PathPlanner& p) {
                    auto status = _path_sync.checkWaitStatus(p.getId());
                    return status.error == PathSync::SUCCESS ||
                           (status.error == PathSync::REMAINING_DURATION_INFINITE && allow_block);
                })) {
                break;
            }
            ++result.request;
        }
    }
    return PlanResult{nullptr};
}

}  // namespace decentralized_path_auction

#if 0
struct Agent {
    PathSearch path_search;
    Path path;
    Nodes src_candidates;
    float fallback_cost;
    size_t path_id = 0;

    Agent(PathSearch::Config config, Nodes src, Nodes dst, float fb_cost = FLT_MAX, float dst_dur = FLT_MAX)
            : path_search(std::move(config))
            , path{{path_search.selectSource(src)}}
            , src_candidates(std::move(src))
            , fallback_cost(fb_cost) {
        path_search.reset(std::move(dst), dst_dur);
    }

    const std::string& id() { return path_search.editConfig().agent_id; }
};

void multi_iterate(std::vector<Agent>& agents, int rounds, size_t iterations, bool allow_block, bool print = false) {
    PathSync path_sync;
    while (--rounds > 0) {
        for (auto& agent : agents) {
            auto search_error = agent.path_search.iterate(agent.path, iterations, agent.fallback_cost);
            ASSERT_LE(search_error, PathSearch::ITERATIONS_REACHED);
            if (print) {
                printf("%s error %d\r\n", agent.id().c_str(), search_error);
                print_path(agent.path);
            }
            auto update_error = path_sync.updatePath(agent.id(), agent.path, agent.path_id++);
            ASSERT_EQ(update_error, PathSync::SUCCESS);
            if (std::all_of(agents.begin(), agents.end(), [&](Agent& a) {
                    auto error = std::get<0>(path_sync.checkWaitConditions(a.id()));
                    if (error == PathSync::SOURCE_NODE_OUTBID) {
                        a.path = {a.path_search.selectSource(a.src_candidates)};
                    }
                    return error == PathSync::SUCCESS ||
                           (error == PathSync::REMAINING_DURATION_INFINITE && allow_block);
                })) {
                rounds = 0;
                break;
            }
        }
    }
    if (print) {
        save_paths(path_sync, "paths.csv");
    }
    ASSERT_EQ(rounds, -1);
}

bool save_paths(const PathSync& path_sync, const char* file) {
    auto fp = fopen(file, "w");
    if (!fp) {
        return false;
    }
    fprintf(fp, "agent_id, agent_idx, pos_x, pos_y, price, rank\r\n");
    int i = 0;
    for (auto& info : path_sync.getPaths()) {
        for (auto& visit : info.second.path) {
            auto& bids = visit.node->auction.getBids();
            fprintf(fp, "\"%s\", %d, %f, %f, %f, %lu\r\n", info.first.c_str(), i, visit.node->position.get<0>(),
                    visit.node->position.get<1>(), visit.price, std::distance(bids.find(visit.price), bids.end()) - 1);
        }
        ++i;
    }
    return !fclose(fp);
}
#endif
