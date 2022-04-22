#pragma once
#include <decentralized_path_auction/path_search.hpp>
#include <decentralized_path_auction/path_sync.hpp>

namespace decentralized_path_auction {

class PathPlanner {
public:
    // constructor
    PathPlanner(PathSearch::Config config)
            : _path_search(std::move(config)) {}

    // getters
    PathSearch& getPathSearch() { return _path_search; }
    const Path& getPath() const { return _path; }

    struct PlanArgs {
        Nodes src;
        size_t iterations;
        float fallback_cost;
    };

    PathSearch::Error plan(const PlanArgs& args, Nodes dst, float duration = FLT_MAX);
    PathSearch::Error replan(const PlanArgs& args);

private:
    PathSearch _path_search;
    Path _path;
};

class MultiPathPlanner {
    struct Request {
        NodePtr src;
        Nodes dst;
        float duration;
        PathSearch::Config config;
        PathPlanner::PlanArgs args;
    };

public:
    struct PlanResult {
        const Request* request;
        PathSearch::Error search_error = PathSearch::SUCCESS;
        PathSync::Error sync_error = PathSync::SUCCESS;
    };
    PlanResult plan(const std::vector<Request>& requests, size_t rounds, bool allow_block = true);

    const PathSync& getPathSync() const { return _path_sync; }

private:
    PathSync _path_sync;
    std::vector<PathPlanner> _path_planners;
};

}  // namespace decentralized_path_auction
