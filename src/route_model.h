
#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        Node* parent = nullptr;     // previous node on the path
        float h_value = std::numeric_limits<float>::max();  // heuristic func, e.g., manhattan distance from curr to goal
        float g_value = 0.0;    // cost up to current node
        bool visited = false;
        std::vector<Node*> neighbors;   // neighboring nodes (on different roads)


        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

        // distance between this node and the other
        float distance(const Node& node) const {
            return std::sqrt(std::pow(x - node.x, 2) + std::pow(y - node.y, 2));
        }

        // populates ‘neighbors’ variable
        void FindNeighbors();

      private:
        int index;      // id for this node
        RouteModel * parent_model = nullptr;    // the RouteModel this node belongs to
        Node* FindNeighbor(std::vector<int> node_indices);  // find closest unvisited node from a given vector of node indices
    };


    RouteModel(const std::vector<std::byte> &xml);  // RouteModel constructor
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

    auto& SNodes() { return m_Nodes; }

    auto& GetNodeToRoadMap() { return node_to_road; }

    Node& FindClosestNode(float x, float y);    // the closest valid node to user input coordinate

  private:
    std::vector<Node> m_Nodes;      // all RouteModel::Nodes from OSM data
    std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;  // the node is on the roads that the road pointers point to

    void CreateNodeToRoadHashMap();     // populates node_to_road
};

#endif
