#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node   = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h = node->distance(*end_node);
    return(h);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors();

    for (RouteModel::Node * neighbor_node : current_node->neighbors) {
        neighbor_node->parent = current_node;
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value + neighbor_node->distance(*current_node);
        neighbor_node->visited = true;
        this->open_list.push_back(neighbor_node);
    }
    current_node->visited = true;

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

/*
RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node* next_node = nullptr;
    float lowest = -1;
    for (RouteModel::Node* node : open_list) {
        float cost = node->g_value + node->h_value;
        if ( cost < lowest || lowest == -1) {
            next_node = node;
            lowest = cost;
        }
        open_list.pop_back();
//        std::cout << "DEBUG: cost " << cost << "\n";
//        std::cout << "DEBUG: lowest " << lowest << "\n";
//        std::cout << "DEBUG: node " << node << "\n";
//        std::cout << "DEBUG: next_node " << next_node << "\n";
    }
    return(next_node);
}
*/


bool Compare(RouteModel::Node* a, RouteModel::Node* b) {
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    //    for (int i = 0; i < open_list.size(); i++) {
    //             float cost = open_list[i]->g_value + open_list[i]->h_value;
    //             std::cout << "DEBUG: path_stack i " << i << " " << open_list[i] << "cost: " << cost << "\n";
    //     }
    RouteModel::Node* next_node = open_list.back();
    // std::cout << "DEBUG: Next node " << next_node << "\n";
    open_list.pop_back();
    
    // std::cout << "DEBUG: next_node " << next_node << "\n";
    
    return(next_node);
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    std::vector<RouteModel::Node *>  path_stack;
    // RouteModel::Node *parent_node = current_node->parent;
    while (current_node->x != start_node->x && current_node->y != start_node->y) {
        distance += current_node->distance(*(current_node->parent));
        // std::cout << "DEBUG: distance " << distance << "\n";
        // std::cout << "DEBUG: current_node x y " << current_node->x << "," << current_node->y << "\n";
        path_stack.push_back(current_node);
        // std::cout << "DEBUG: push current_node " << current_node << "\n";
        // std::cout << "DEBUG: parent_node  " << current_node->parent << "\n";
        // for (int i = 0; i < path_stack.size(); i++) {
        //         std::cout << "DEBUG: path_stack i " << i << " " << path_stack[i] << "\n";
        // }
        current_node = current_node->parent;
    }
    // push the start node onto th stack
    path_stack.push_back(current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    // std::cout << "DEBUG: distance " << distance << "\n";

    RouteModel::Node *tmp;
    while(!path_stack.empty()) {
        tmp = path_stack.back();
        // std::cout << "DEBUG: tmp x y " << tmp->x << "," << tmp->y << "\n";
        // std::cout << "DEBUG: tmp " << tmp << "\n";
        path_stack.pop_back();
        path_found.push_back(*tmp);
    }
    
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->parent = nullptr;
    start_node->h_value = CalculateHValue(start_node);
    start_node->g_value = 0;
    start_node->visited = true;
    AddNeighbors(start_node);

    // std::cout << "DEBUG: end_node " << end_node << "(" << end_node->x << "," << end_node->y << ")" << "\n";
    // std::cout << "DEBUG: open_list.size() " << open_list.size() << "\n";
    int debug = 0;
    while(open_list.size()){
        current_node = NextNode();
        // std::cout << "DEBUG: current_node " << current_node << "(" << current_node->x << "," << current_node->y << ")" << "\n";
        // std::cout << "DEBUG: open_list.size() " << open_list.size() << "\n";
        //if (debug > 4) { return; } else { ++debug; }
        if (current_node->x == end_node->x && current_node->y == end_node->y){
            // reached endpoint
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
    std::cout << "ERROR: No Path Found!\n";
    return;
}