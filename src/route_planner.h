#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);    // Constructor
    float GetDistance() const { return distance; }
    // perform A* search
    void AStarSearch();

  private:

    RouteModel &m_Model;    // map model
    RouteModel::Node *start_node, *end_node;
    float distance;         // total path distance from start to end
    //TODO: make open_list a priority queue to improve performance
    std::vector<RouteModel::Node*> open_list;

    // construct path from start node to end node found by AStarSearch, starting from end node and then backwards
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node);

    // heuristic function value
    float CalculateHValue(const RouteModel::Node *);


    RouteModel::Node* NextNode();
    void AddNeighbors(RouteModel::Node *);
};

#endif
