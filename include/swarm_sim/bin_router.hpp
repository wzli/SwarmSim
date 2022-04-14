#pragma once
#include <decentralized_path_auction/path_search.hpp>
#include <decentralized_path_auction/path_sync.hpp>

#include <string>
#include <unordered_map>

namespace swarm_sim {
using namespace decentralized_path_auction;

struct Bin {
    Path path;
    PathSearch path_search;
    PathSync::Status path_status{};
    size_t path_id = 0;
};

class BinRouter {
    using Bins = std::unordered_map<std::string, Bin>;
    using BinQueue = std::vector<std::string>;

    enum Error {
        SUCCESS,
    };

public:
    Error updateBinNode(std::string_view id, NodePtr node);
    Error requestBinNode(std::string_view id, NodePtr node);

    const Bins& getBins() const { return _bins; }
    const BinQueue& getBinQueue() const { return _bin_queue; }

private:
    void updatePaths();

    Bins _bins;
    BinQueue _bin_queue;
    PathSync _path_sync;
};

}  // namespace swarm_sim
