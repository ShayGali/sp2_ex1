#pragma once
#include "Graph.hpp"

/**
 * @brief an concrete class that represents an undirected graph as an adjacency matrix
 */
class UndirectedGraph : public Graph {
   public:
    UndirectedGraph();
   /**
    * @brief Construct a new Undirected Graph object
    * will update by the upper half of the matrix
   */
    void loadGraph(vector<vector<int>> ajdList) override;
    void printGraph() override;
};