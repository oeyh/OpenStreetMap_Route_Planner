#include "route_planner.h"
#include <algorithm>

// RoutePlanner constructor
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // scale float numbers to percentages
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}

// construct path from start node to end node found by AStarSearch, starting from end node and then backwards
// current node in the argument is the end node
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    distance = 0.0f;  // if f is not specified, it will be of default type -- double
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *temp_node = current_node;
    while (temp_node->parent != nullptr) {
        path_found.push_back(*temp_node);
        distance += temp_node->distance(*temp_node->parent);
        temp_node = temp_node->parent;
    }
    path_found.push_back(*temp_node);
    distance *= m_Model.MetricScale();  // scale the distance to meters
    return path_found;
}

// perform A* search
void RoutePlanner::AStarSearch() {
    // init
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    // A star search
    while (!open_list.empty()) {
        // find next node with min f value
        current_node = NextNode();

        // check if end node is reached
        if (current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }

}

// compute h-value from heuristic function
// here it is defined as Euclidean distance from this node to end node
float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
    // H value here is the direct distance
    return node->distance(*end_node);

}

// get next node (with smallest f-value) to process
RouteModel::Node* RoutePlanner::NextNode() {
    // sorting custom objects
    // I sorted descending here so the smallest node is at the back of the vector
    std::sort(open_list.begin(), open_list.end(), [](const auto &lhs, const auto &rhs) {
        return (lhs->g_value + lhs->h_value) > (rhs->g_value + rhs->h_value);
    });

    RouteModel::Node *next_node = open_list[open_list.size() - 1];
    open_list.pop_back();
    return next_node;
}

// Add neighboring nodes to open_list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors(); // this populates current_node's neighbors vector

    // update neighboring node properties and add to open_list
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);

        open_list.push_back(neighbor);
        neighbor->visited = true;
    }

}
