#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    // Return the distance to another node.
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    // Populate current_node with all neighbors.
    current_node->FindNeighbors();

    for (auto neighbor_node : current_node->neighbors)
    {
        neighbor_node->parent = current_node;

        // For each node in current_node, set the parent (h_value and the g_value). 
        neighbor_node->g_value = current_node->g_value + current_node->distance(*neighbor_node);
        
        // h-Value calculation.
        neighbor_node->h_value = CalculateHValue(neighbor_node);

        // Add the neighbor to open_list.
        open_list.push_back(neighbor_node);

        // Set the node's visited attribute to true.
        neighbor_node->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd) {
        return (_1st->h_value + _1st->g_value) < (_2nd->h_value + _2nd->g_value);
    });

    // Get the node with lowest sum.
    RouteModel::Node *lowest_sum_node = open_list.front();

    // Remove node from the open_list.
    open_list.erase(open_list.begin());

    return lowest_sum_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Repeat until find the starting point.
    while (current_node->parent != nullptr)
    {
        // For each node in the chain, add the distance from the node to its parent to the distance variable.
        path_found.emplace_back(*current_node);
        const RouteModel::Node parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }

    // Add start node in path found.
    path_found.push_back(*start_node);
    
    // Reverse the order of the elements in the range (Start node should be the first element, the end node should be the last element).
    std::reverse(path_found.begin(), path_found.end());

    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale(); 

    return path_found;
}

void RoutePlanner::AStarSearch()
{
    // Start with starting node.
    start_node->visited = true;
    open_list.emplace_back(start_node);

    while (!open_list.empty())
    {
        // Find the next node.
        RouteModel::Node *current_node = NextNode();

        // Check if reached the goal.
        if (current_node->distance(*end_node) == 0)
        {
            // Get the final path that was found and store in the m_Model.path
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }

        // Add current node to the neighbors.
        AddNeighbors(current_node);
    }
}