#pragma once

#include <vector>
using namespace std;

namespace ariel {
#define NO_EDGE 0
class Graph {
   private:
    vector<vector<int>> ajdList;
    bool isWeighted = false;
    bool haveNegativeEdgeWeight = false;

   public:
    /**
     * @brief Construct a new Graph object. create a deep copy of the input matrix
     * @throw invalid_argument if the diagonal of the matrix is not 0
     */
    void loadGraph(vector<vector<int>>);

    /**
     * @brief Print data about the graph
     * the format is: "Graph with <number of vertices> vertices and <number of edges> edges."
     */
    void printGraph();
    vector<vector<int>> getGraph();
    bool isWeightedGraph();
    bool isHaveNegativeEdgeWeight();
};
}  // namespace ariel
