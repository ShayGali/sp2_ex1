#pragma once

#include <limits>
#include <vector>

using std::vector;

#define INF std::numeric_limits<int>::max()  // represent infinity
#define NO_EDGE INF                          // represent no edge between two vertices as infinity

namespace shayg {

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

    /**
     * @brief Load the graph from an adjacency matrix
     * @param ajdList the adjacency matrix
     * @throw invalid_argument if the graph is not a square matrix or the diagonal of the matrix is not NO_EDGE
     * @throw invalid_argument if the graph is undirected and the matrix is not symmetric
     */
    void loadGraph(vector<vector<int>> ajdList);

    /**
     * @brief Print the graph as an adjacency matrix
     * the output will be like this:
     * "Directed/Undirected graph with |V| vertices and |E| edges."
     * where |V| is the number of vertices and |E| is the number of edges.
     */
    void printGraph();

    /**
     * @brief return shallow copy of the adjacency matrix
     * @return vector<vector<int>> the adjacency matrix
     */
    vector<vector<int>> getGraph();

    /**
     * @brief check if the graph is directed
     * @return true if the graph is directed, false otherwise
     */
    bool isDirectedGraph();

    /**
     * @brief check if the graph is weighted
     * @return true if the graph is weighted, false otherwise
     */
    bool isWeightedGraph();

    /**
     * @brief check if the graph has negative edge weight
     * @return true if the graph has negative edge weight, false otherwise
     */
    bool isHaveNegativeEdgeWeight();
};

}  // namespace shayg