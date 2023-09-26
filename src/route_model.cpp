#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes.
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}


void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}


RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}


void RouteModel::Node::FindNeighbors() {
    for (auto & road : parent_model->node_to_road[this->index]) {
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}


RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    
    // Create a Node object `input` and assign `x` and `y` to it.
    Node input;
    input.x = x;
    input.y = y;


    // set the initial min_dist to the maximum possible value
    float min_dist = std::numeric_limits<float>::max();

    // Declare variables dist and closest_idx
    // Declare a variable `dist` to hold the distance between `input` and the nodes we will examine.
    float dist;
    // Declare an integer `closest_idx` to hold the index of the closest node.
    int closest_idx;

    // Iterate over all the roads in the model
    for (const Model::Road &road : Roads()) {
        // If the road is not a footway (as we want to ignore footways)
        if (road.type != Model::Road::Type::Footway) {
            // Iterate over the indices of all nodes in the way corresponding to the current road.
            for (int node_idx : Ways()[road.way].nodes) {
                // Calculate the distance between `input` node and the current node
                dist = input.distance(SNodes()[node_idx]);
                 // If this distance is less than the current `min_dist`,
                if (dist < min_dist) {
                    // update `min_dist` and `closest_idx` with the new minimum distance and corresponding node index.
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }
    // After iterating over all nodes, return a reference to the node with the smallest distance to `input`, 
    // i.e., the node at index `closest_idx`.
    return SNodes()[closest_idx];
}