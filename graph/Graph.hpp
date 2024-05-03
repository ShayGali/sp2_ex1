#pragma once

#include <vector>

using std::vector;

#define INF std::numeric_limits<int>::max()  // represent infinity
#define NO_EDGE INF                          // represent no edge between two vertices as infinity

/**
 * @brief an abstract class that represents a graph as an adjacency matrix
 */
class Graph {
   private:
    vector<vector<int>> ajdList;
    bool isDirected;
    bool isWeighted;
    bool haveNegativeEdgeWeight;

   public:
    /**
     * @brief Construct a new Graph object
     * @param isDirected whether the graph is directed or not. Default is false.
     */
    Graph(bool isDirected = false);

    void loadGraph(vector<vector<int>> ajdList);
    void printGraph();

    vector<vector<int>> getGraph();
    bool isDirectedGraph();
    bool isWeightedGraph();
    bool isHaveNegativeEdgeWeight();
};