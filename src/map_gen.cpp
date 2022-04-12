#include <swarm_sim/map_gen.hpp>

namespace swarm_sim {

MapGen::MapGen(const Config& config) {
    // add a grid of nodes to graph
    std::vector<NodePtr> nodes;
    for(size_t i = 0; i < config.cols; ++i) {
        for(size_t j = 0; j < config.rows; ++j) {
            nodes.emplace_back(graph.insertNode(Point{i * config.spacing,
                        j * config.spacing}, Node::DEFAULT));
        }
    }
    // add edges to grid of nodes
    for(size_t i = 0; i < config.cols; ++i) {
        for(size_t j = 0; j < config.rows; ++j) {
            auto& node = nodes[i + (j * config.cols)];
            if(i > 0) {
                node->edges.push_back(nodes[(i - 1) + (j * config.cols)]);
            }
            if(i < config.cols - 1) {
                node->edges.push_back(nodes[(i + 1) + (j * config.cols)]);
            }
            if(j > 0) {
                node->edges.push_back(nodes[i + ((j - 1) * config.cols)]);
            }
            if(j < config.rows - 1) {
                node->edges.push_back(nodes[i + ((j + 1) * config.cols)]);
            }
        }
    }
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

}

