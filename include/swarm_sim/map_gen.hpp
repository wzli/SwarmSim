#pragma once
#include <decentralized_path_auction/graph.hpp>
#include <vector>

namespace swarm_sim {

using namespace decentralized_path_auction;

struct MapGen {
    // create a grid map
    void generate_graph(size_t rows, size_t cols, float spacing = 1.0f) {
        graph.clearNodes();
        std::vector<NodePtr> nodes;
        for(size_t i = 0; i < cols; ++i) {
            for(size_t j = 0; j < rows; ++j) {
                nodes.emplace_back(graph.insertNode(Point{i * spacing, j * spacing}, Node::DEFAULT));
            }
        }
        for(size_t i = 0; i < cols; ++i) {
            for(size_t j = 0; j < rows; ++j) {
                auto& node = nodes[i + (j * cols)];
                if(i > 0) {
                    node->edges.push_back(nodes[(i - 1) + (j * cols)]);
                }
                if(i < cols - 1) {
                    node->edges.push_back(nodes[(i + 1) + (j * cols)]);
                }
                if(j > 0) {
                    node->edges.push_back(nodes[i + ((j - 1) * cols)]);
                }
                if(j < rows - 1) {
                    node->edges.push_back(nodes[i + ((j + 1) * cols)]);
                }
            }
        }
    }

    // populate with random boxes

    // populate with random robots
    Graph graph;
};

}
