#include <string>

#include "../graph/DirectedGraph.hpp"
#include "../graph/Graph.hpp"
#include "../graph/UndirectedGraph.hpp"

using std::string;

// enum to represent the colors of the vertices (use for DFS and bipartite)
enum Color {
    // for DFS
    WHITE,
    GRAY,
    BLACK,

    // for bipartite
    BLUE,
    RED
};

class Algorithms {
   public:
    /**
     * @brief Check if the graph is connected
     * @param g - the graph to check
     * @return true if the graph is connected, false otherwise
     */
    static bool isConnected(DirectedGraph& g);
    static bool isConnected(UndirectedGraph& g);

    /**
     * @brief Find the shortest path between two vertices
     * @param g - the graph to search in
     * @param src - the source vertex
     * @param dest - the destination vertex
     * @return the shortest path between the source and destination vertices in the format "src->...->dest".
     *  if there is no path between the source and destination vertices, return "-1"

     */
    static string shortestPath(Graph& g, size_t src, size_t dest);

    /**
     * @brief Check if the graph contains a cycle
     * @param g - the graph to check
     * @return true if the graph contains a cycle, false otherwise
     */
    static bool isContainsCycle(Graph& g);

    /**
     * @brief Check if the graph is bipartite
     * @param g - the graph to check
     * @return If the graph is bipartite, return "The graph is bipartite: A={...}, B={...}". (the ... represents the vertices in the set)
     *     Otherwise, return "0".
     */
    static string isBipartite(Graph& g);

    /**
     * @brief Find the negative cycle in the graph
     * @param g - the graph to search in
     * @return If the graph contains a negative cycle, return the cycle in the format "v1->v2->...->v1".
     *    Otherwise, return "0".
     *
     */
    static string negativeCycle(DirectedGraph& g);
    static string negativeCycle(UndirectedGraph& g);

    class NegativeCycleException : public std::exception {
       public:
        size_t detectedCycleStart;
        vector<int> parentList;
        NegativeCycleException(size_t detectedCycleStart, vector<int> parentList) {
            this->detectedCycleStart = detectedCycleStart;
            this->parentList = parentList;
        }

        virtual const char* what() const throw() {
            return "Graph contains a negative-weight cycle";
        }
    };
};
