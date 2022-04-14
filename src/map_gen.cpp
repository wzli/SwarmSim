#include <swarm_sim/map_gen.hpp>
#include <algorithm>

namespace swarm_sim {

MapGen::MapGen(const Config& config) {
    // helper to convert indices
    const auto idx = [&config](size_t col, size_t row = 0, size_t flr = 0) {
        return col + (row * config.cols) + (flr * config.cols * config.rows);
    };
    // create a bit vector of elevator positions
    std::vector<bool> has_elevator;
    has_elevator.resize(config.cols * config.rows);
    for (auto& [col, row] : config.elevators) {
        has_elevator[idx(col, row)] = true;
    }
    // add 3D grid of nodes to graph
    std::vector<NodePtr> nodes;
    for (size_t flr = 0; flr < config.floors; ++flr) {
        for (size_t row = 0; row < config.rows; ++row) {
            for (size_t col = 0; col < config.cols; ++col) {
                auto node =
                        has_elevator[idx(col, row)]
                                // make elevator node only for the first floor
                                ? flr == 0 ? graph.insertNode(Point{static_cast<float>(col),
                                                                      static_cast<float>(row), 0},
                                                     Node::NO_STOPPING)
                                           // every other floor with elevator references the first
                                           : nodes[idx(col, row)]
                                // if not an elevator add a new node
                                : graph.insertNode(
                                          Point{static_cast<float>(col), static_cast<float>(row),
                                                  static_cast<float>(flr)},
                                          Node::DEFAULT);
                nodes.emplace_back(node);
            }
        }
    }
    // add edges to grid of nodes
    for (size_t flr = 0; flr < config.floors; ++flr) {
        for (size_t row = 0; row < config.rows; ++row) {
            for (size_t col = 0; col < config.cols; ++col) {
                auto& node = nodes[idx(col, row, flr)];
                if (col > 0) {
                    node->edges.push_back(nodes[idx(col - 1, row, flr)]);
                }
                if (col < config.cols - 1) {
                    node->edges.push_back(nodes[idx(col + 1, row, flr)]);
                }
                if (row > 0) {
                    node->edges.push_back(nodes[idx(col, row - 1, flr)]);
                }
                if (row < config.rows - 1) {
                    node->edges.push_back(nodes[idx(col, row + 1, flr)]);
                }
            }
        }
    }
    // remove elevator nodes first list
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                        [](const NodePtr& x) { return x->state == Node::NO_STOPPING; }),
            nodes.end());

    // shuffle the index of nodes
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(nodes.begin(), nodes.end(), gen);

    size_t n_bins = std::min(config.n_bins, nodes.size());
    size_t n_bots = std::min(config.n_bots, nodes.size() - n_bins);

    // assign random grid positions to bins and bots
    bots.assign(nodes.end() - n_bots, nodes.end());
    bins = std::move(nodes);
    bins.resize(n_bins);
}

}  // namespace swarm_sim
