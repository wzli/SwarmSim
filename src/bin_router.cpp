#include <swarm_sim/bin_router.hpp>
#include <algorithm>

namespace swarm_sim {

BinRouter::Error BinRouter::updateBinNode(std::string_view id, NodePtr node) {
    auto [bin, inserted] = _bins.emplace(id, PathPlanner(PathSearch::Config{std::string(id)}));
    // bid did not exist before, now it is added
    if (inserted) {
        // TODO: set as fallback path on path sync
        return SUCCESS;
    }
    // if update node is on current path update route sync progress
    auto& path = bin->second.getPath();
    auto found = std::find_if(
            path.begin(), path.end(), [&node](auto visit) { return visit.node == node; });
    if (found == path.end()) {
        // todo update path_sync progress
    }
    // TODO: if update is not on current path, query for new path then update path_sync
    return SUCCESS;
}

}  // namespace swarm_sim
