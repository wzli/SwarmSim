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

PathSearch::Error MultiPathPlanner::plan(
        const Config& config, const std::vector<Request>& requests) {
    _path_sync.clearPaths();
    _path_planners.clear();
    _results.clear();
    // initialize path planners and set destination
    for (auto& req : requests) {
        _path_planners.emplace_back(req.config);
        _results.emplace_back(Result{
                _path_planners.back().getPathSearch().setDestinations(req.dst, req.duration)});
        if (_results.back().search_error) {
            return _results.back().search_error;
        }
    }

    // setup thread shared data
    _countdown = static_cast<int>(config.rounds * requests.size());
    _requests = &requests[0];
    _config = config;
    // cannot have more threads than there are paths
    _config.n_threads = std::min(config.n_threads, requests.size());

    // spawn threads
    std::vector<std::thread> threads;
    for (size_t thread_idx = 0; thread_idx < _config.n_threads; ++thread_idx) {
        threads.emplace_back(&MultiPathPlanner::thread_loop, this, thread_idx);
    }

    // wait for threads completion
    for (auto& thread : threads) {
        thread.join();
    }
    return static_cast<PathSearch::Error>(-_countdown);
}

void MultiPathPlanner::thread_loop(size_t idx) {
    size_t path_id = 0;
    while (true) {
        auto& request = _requests[idx];
        auto& planner = _path_planners[idx];
        auto& result = _results[idx];
        PathSearch::Error search_error;
        // replan path only requires read access
        {
            std::shared_lock lock(_shared_mutex);
            if (_countdown <= 0) {
                return;
            }
            search_error = planner.replan(request.args);
            /*
            printf("idx %d e %d p %f c %f fb %f\n", idx, search_error,
                    planner.getPath().back().price, planner.getPath().back().cost_estimate,
                    request.args.fallback_cost);
            */
        }
        // enter write lock
        {
            std::unique_lock lock(_shared_mutex);
            if (_countdown <= 0) {
                return;
            }
            --_countdown;
            if (!_countdown) {
                printf("out of iter\n");
            }

            result = {search_error};
            // terminate search if error occurered
            if (search_error > PathSearch::ITERATIONS_REACHED) {
                _countdown = -search_error;
                printf("search error %d\n", search_error);
                return;
            }
            // otherwise add to path sync
            result.sync_error =
                    _path_sync.updatePath(planner.getId(), planner.getPath(), path_id++);

            // terminate when all paths are satisfactory
            if (std::all_of(_path_planners.begin(), _path_planners.end(), [&](PathPlanner& p) {
                    // check if there are any stale fallback paths
                    int path_idx = &p - &_path_planners[0];
                    if ((_requests[path_idx].dst.empty() ||
                                (!p.getPath().empty() &&
                                        _requests[path_idx].dst[0] == p.getPath().front().node)) &&
                            _results[path_idx].search_error == PathSearch::FALLBACK_DIVERTED &&
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
                    auto& error = _results[path_idx].sync_error;
                    error = _path_sync.checkWaitStatus(p.getId()).error;
                    return error == PathSync::SUCCESS ||
                           (error == PathSync::REMAINING_DURATION_INFINITE &&
                                   _config.allow_indefinite_block);
                })) {
                _countdown = 0;
                printf("graceful\n");
                return;
            }
        }

        // select next path to plan
        idx += _config.n_threads;
        if (idx >= _path_planners.size()) {
            idx %= _config.n_threads;
        }
    }
}

}  // namespace decentralized_path_auction
