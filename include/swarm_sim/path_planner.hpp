#pragma once
#include <decentralized_path_auction/path_search.hpp>
#include <decentralized_path_auction/path_sync.hpp>
#include <thread>
#include <shared_mutex>

namespace decentralized_path_auction {

class PathPlanner {
public:
    // constructor
    PathPlanner(PathSearch::Config config)
            : _path_search(std::move(config)) {}

    // getters
    PathSearch& getPathSearch() { return _path_search; }
    const Path& getPath() const { return _path; }
    const std::string& getId() const { return _path_search.getConfig().agent_id; }

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
public:
    struct Config {
        size_t rounds;
        size_t n_threads = std::thread::hardware_concurrency();
        bool allow_indefinite_block = true;
    };

    struct Request {
        Nodes dst;
        float duration;
        PathSearch::Config config;
        PathPlanner::PlanArgs args;
    };

    struct Result {
        PathSearch::Error search_error = PathSearch::SUCCESS;
        PathSync::Error sync_error = PathSync::SUCCESS;
    };

    PathSearch::Error plan(const Config& config, const std::vector<Request>& requests);

    const PathSync& getPathSync() const { return _path_sync; }
    PathSync& getPathSync() { return _path_sync; }

    const std::vector<Result>& getResults() const { return _results; }

private:
    void thread_loop(size_t idx);

    PathSync _path_sync;
    std::vector<PathPlanner> _path_planners;
    std::vector<Result> _results;
    const Request* _requests;
    Config _config;

    std::atomic<int> _countdown;
    std::shared_mutex _shared_mutex;
};

}  // namespace decentralized_path_auction
