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
        const std::vector<Request>& requests, int rounds, bool allow_block) {
    _path_sync.clearPaths();
    _path_planners.clear();
    // initialize path planners and set destination
    for (auto& req : requests) {
        _path_planners.emplace_back(req.config);
        PlanResult result{
                &req, _path_planners.back().getPathSearch().setDestinations(req.dst, req.duration)};
        if (result.search_error) {
            return result;
        }
    }
    std::vector<PathSearch::Error> search_errors(requests.size());
    size_t path_id = 0;
    while (--rounds > 0) {
        PlanResult result{&requests.front()};
        size_t path_idx = 0;
        for (auto& planner : _path_planners) {
            // replan path
            result.search_error = search_errors[path_idx] = planner.replan(result.request->args);
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
                    // check if there are any stale fallback paths
                    int path_idx = &p - &_path_planners[0];
                    if (p.getPath().size() > 1 &&
                            search_errors[path_idx] == PathSearch::FALLBACK_DIVERTED &&
                            std::next(p.getPath().front().node->auction.getBids().begin())
                                            ->second.bidder == p.getId()) {
                        p.getPathSearch().resetCostEstimates();
                        return false;
                    }
                    auto status = _path_sync.checkWaitStatus(p.getId());
                    return status.error == PathSync::SUCCESS ||
                           (status.error == PathSync::REMAINING_DURATION_INFINITE && allow_block);
                })) {
                rounds = 0;
                break;
            }
            ++result.request;
            ++path_idx;
        }
    }
    return PlanResult{nullptr, rounds ? PathSearch::SUCCESS : PathSearch::ITERATIONS_REACHED};
}

}  // namespace decentralized_path_auction
