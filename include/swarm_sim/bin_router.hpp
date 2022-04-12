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
    size_t path_id = 0;
};

class BinRouter {
    using Bins = std::unordered_map<std::string, Bin>;
    enum Error {
        SUCCESS,
    };
public:
    Error updateBinNode(std::string_view id, NodePtr node);
    Error requestBinNode(std::string_view id, NodePtr node);

    const Bins& getBins() const { return _bins; }
    // get routes function

private:
    Bins _bins;
    PathSync _path_sync;
};

}
