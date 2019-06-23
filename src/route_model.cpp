#include "route_model.h"
#include <iostream>

// RouteModel constructor
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // create RouteModel nodes from osm data
    for (int i = 0; i < this->Nodes().size(); ++i) {
        m_Nodes.push_back(Node(i, this, this->Nodes()[i]));
    }
    // create mapping from node to roads (pointers) so we know which roads a node belongs to (one node belongs to multiple roads)
    CreateNodeToRoadHashMap();
}

// Node to road mapping creation
void RouteModel::CreateNodeToRoadHashMap() {
    for (const auto& road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road*>();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

// find closest unvisited node from a given vector of node indices
RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node* closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr ||
                this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

// find all neighbor nodes (one from each road that this node belongs to)
void RouteModel::Node::FindNeighbors() {
    for (auto& road : parent_model->node_to_road[this->index]) {
        Node* node_ptr = FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (node_ptr) {
            this->neighbors.push_back(node_ptr);
        }
    }
}

// given x and y coordinates (e.g. from user input), find the closest node on the map (because x and y may not be exactly on a node)
RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
    Node user_node;
    user_node.x = x;
    user_node.y = y;

    float min_dist = std::numeric_limits<float>::max();
    int closest_idx;

    // Look for the closest node that is on a road but not a footway
    // If instead looking for the node on SNodes(), the node may be on a closed path or isolated
    for (auto& road : this->Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int idx : Ways()[road.way].nodes) {
                float curr_dist = user_node.distance(this->SNodes()[idx]);
                if (curr_dist < min_dist) {
                    min_dist = curr_dist;
                    closest_idx = idx;
                }
            }
        }
    }
    return this->SNodes()[closest_idx];

}
