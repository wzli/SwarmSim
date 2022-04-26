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

MultiPathPlanner::Results MultiPathPlanner::plan(
        const Config& config, const std::vector<Request>& requests) {
    _path_sync.clearPaths();
    _path_planners.clear();
    Results results;
    results.reserve(requests.size());
    // initialize path planners and set destination
    for (auto& req : requests) {
        _path_planners.emplace_back(req.config);
        results.emplace_back(Result{
                _path_planners.back().getPathSearch().setDestinations(req.dst, req.duration)});
        if (results.back().search_error) {
            return results;
        }
    }
    std::vector<PathSearch::Error> search_errors(requests.size());
    size_t path_id = 0;
    for (size_t round = config.rounds; round > 0; --round) {
        auto request = &requests.front();
        auto result = &results.front();
        for (auto& planner : _path_planners) {
            *result = {};
            // replan path
            result->search_error = planner.replan(request->args);
            if (result->search_error > PathSearch::ITERATIONS_REACHED) {
                return results;
            }
            // add path to path_sync
            result->sync_error =
                    _path_sync.updatePath(planner.getId(), planner.getPath(), path_id++);
            if (result->sync_error) {
                return results;
            }
            // terminate when all paths satisfactory
            if (std::all_of(_path_planners.begin(), _path_planners.end(), [&](PathPlanner& p) {
                    // check if there are any stale fallback paths
                    int path_idx = &p - &_path_planners[0];
                    if (results[path_idx].search_error == PathSearch::FALLBACK_DIVERTED &&
                            std::any_of(p.getPath().begin(), p.getPath().end() - 1,
                                    [&p](const Visit& visit) {
                                        return visit.node->state < Node::NO_PARKING &&
                                               std::next(visit.node->auction.getBids().begin())
                                                               ->second.bidder == p.getId();
                                    })) {
                        p.getPathSearch().resetCostEstimates();
                        return false;
                    }
                    // check paths are compatible with each other
                    auto& error = results[path_idx].sync_error;
                    error = _path_sync.checkWaitStatus(p.getId()).error;
                    return error == PathSync::SUCCESS ||
                           (error == PathSync::REMAINING_DURATION_INFINITE &&
                                   config.allow_indefinite_block);
                })) {
                round = 1;
                break;
            }
            ++request;
            ++result;
        }
    }
    return results;
}

}  // namespace decentralized_path_auction
