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
	start_node = &(model.FindClosestNode(start_x, start_y));
  	end_node = &(model.FindClosestNode(end_x, end_y));
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
  	for(auto node: current_node->neighbors)
    {
      if(!node->visited)
      {
        node->parent = current_node;
        node->g_value = node->distance(*current_node) + current_node->g_value;
        node->h_value = RoutePlanner::CalculateHValue(node);
        node->visited = true;
        open_list.push_back(node);
      }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(),open_list.end(),[](RouteModel::Node* node1, RouteModel::Node* node2)
              {
                return (node1->g_value + node1->h_value) < (node2->g_value + node2->h_value);
              }
             );
  RouteModel::Node* selected_node = open_list[0];
  open_list.erase(open_list.begin());
  return selected_node;
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
  	RouteModel::Node *node = current_node;
   
	while(node != start_node)
    {
      distance += node->distance(*(node->parent));
      path_found.push_back(*node);
      node = node->parent;
    }
     path_found.push_back(*node);
  	reverse(path_found.begin(), path_found.end()); 
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
  	open_list.push_back(start_node);
  	start_node->visited = true;
  while(open_list.size() > 0 && current_node != end_node)
  {
    current_node = RoutePlanner::NextNode();
    RoutePlanner::AddNeighbors(current_node);
    
  }
//   std::cout << current_node->h_value << std::endl;
  m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
}