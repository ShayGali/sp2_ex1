#pragma once

#include "Graph.hpp"

/**
 * @brief an concrete class that represents an directed graph as an adjacency matrix
 */
class DirectedGraph : public Graph {
   public:
    DirectedGraph();
    void loadGraph(vector<vector<int>> ajdList);
    void printGraph();
};