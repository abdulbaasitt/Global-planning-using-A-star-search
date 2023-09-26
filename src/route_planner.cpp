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

    // closest node to the starting coordinates using the m_Model.FindClosestNode method
    start_node = &m_Model.FindClosestNode(start_x, start_y);

    // closest node to the ending coordinates using the m_Model.FindClosestNode method
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    // return  (*node).distance(*end_node);
    // using the distance method from node sub class in the RouteModel Class
    return node->distance(*end_node);

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // This line calls the `FindNeighbors` function on the node object pointed to by `current_node`.
    // The `FindNeighbors` method from the RouteModel Class is used for setting the neighbours of the current node
    current_node->FindNeighbors();
     
     // This is a for-each loop that iterates over each neighbor of the current node. 
    // 'auto&' indicates that `neighbor` is a reference to the element within the `neighbors` vector. 
    // It is used here to avoid unnecessary copying and to allow direct modification of the elements.
    for (auto& neighbor : current_node->neighbors) {

        // the parent of the neighboring node is set to be the current node. 
        neighbor->parent = current_node;
        // The `CalculateHValue` function calculates the heuristic value of the node
        neighbor->h_value = CalculateHValue(neighbor);
        //  the g value of the neighboring node is set to be the g value of current  + distance to neighbor
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        // The neighboring node is added to the open list and its visited attribute is set to true.
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

//  used as a helper function for sorting the open list
bool compare(RouteModel::Node* a, RouteModel::Node* b){

    // the sum of the h value and g value for node a
    float f1 = a->g_value + a->h_value;
    // the sum of the h value and g value for node b
    float f2 = b->g_value + b->h_value;

    return f1 > f2;
}

// the NextNode method to sort the open list and return the next node.
RouteModel::Node* RoutePlanner::NextNode() {

// Sort the open_list according to the sum of the h value and g value.(using the compare function)
std::sort(open_list.begin(), open_list.end(), compare);
//  set the pointer to the node in the list with the lowest sum.
RouteModel::Node* lowest_sum = open_list.back();
// Remove that node from the open_list.
open_list.pop_back();
return lowest_sum;
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
    // iteratively follow the chain of parents of nodes until the starting node is found.
    while (current_node != start_node){

        // add the current node to the path found vector
        path_found.push_back(*current_node);
        // add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*current_node->parent);
        // set the current node to be the parent of the current node(update the current node)
        current_node = current_node->parent;
    }
    // add the start node to the path found vector
    path_found.push_back(*current_node);

    // reverse the path found vector to be in the correct order
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
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
    // set the start node to be visited and add it to the open list
    start_node->visited = true;
    open_list.push_back(start_node);

    // while the open list is not empty(the size of open list > 0 ==> there are nodes to be explored)
    while (!open_list.empty()){

        // set current node to be the next node. 
        // this will sort the open list and the node with the lowest cost
        current_node = NextNode();
        // if the current node is the end node, then the search has reached the end_node
        if (current_node == end_node){
            // use the ConstructFinalPath method to return the final path that was found.
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else{
            // if the current node is not the end node, 
            // then add all of the neighbors of the current node to the open_list.
            AddNeighbors(current_node);
        }
    }

}