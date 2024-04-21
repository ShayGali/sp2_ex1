#pragma once

#include <vector>
using std::vector;

#define INF std::numeric_limits<int>::max()  // represent infinity
#define NO_EDGE INF                          // represent no edge between two vertices as infinity

/**
 * @brief an abstract class that represents a graph as an adjacency matrix
 */
class Graph {
   public:
    Graph();
    virtual vector<vector<int>> getGraph();
    bool isWeightedGraph();
    bool isHaveNegativeEdgeWeight();

    // abstract methods

    /**
     * @brief load the graph from an adjacency list.
    */
    virtual void loadGraph(vector<vector<int>> ajdList) = 0;
    virtual void printGraph() = 0;

   protected:
    vector<vector<int>> ajdList;
    bool isWeighted;
    bool haveNegativeEdgeWeight;
};