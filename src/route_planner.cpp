#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

   
	start_node = &(model.FindClosestNode(start_x, start_y));
  	end_node = &(model.FindClosestNode(end_x, end_y));
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}



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
        open_list.emplace_back(node);
      }
    }
}



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



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	
    // TODO: Implement your solution here.
  	RouteModel::Node *node = current_node;
   
	while(node != start_node)
    {
      distance += node->distance(*(node->parent));
      path_found.emplace_back(*node);
      node = node->parent;
    }
     path_found.emplace_back(*node);
  	reverse(path_found.begin(), path_found.end()); 
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
  	open_list.emplace_back(start_node);
  	start_node->visited = true;
  while(open_list.size() > 0 && current_node != end_node)
  {
    current_node = RoutePlanner::NextNode();
    RoutePlanner::AddNeighbors(current_node);
    
  }
//   std::cout << current_node->h_value << std::endl;
  m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
}