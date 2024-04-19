#include <string>

#include "Graph.hpp"
namespace ariel {

// define a value to represent INFINITY
#define INFINITY std::numeric_limits<int>::max()

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
    static bool isConnected(Graph& g);

    /**
     * @brief Find the shortest path between two vertices
     * @param g - the graph to search in
     * @param src - the source vertex
     * @param dest - the destination vertex
     * @return the shortest path between the source and destination vertices in the format "src->...->dest".
     *  if there is no path between the source and destination vertices, return "-1"

     */
    static string shortestPath(Graph& g, int src, int dest);

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
    static string negativeCycle(Graph& g);

    class NegativeCycleException : public exception {
       public:
        int detectedCycleStart;
        vector<int> parentList;
        NegativeCycleException(int detectedCycleStart, vector<int> parentList) {
            this->detectedCycleStart = detectedCycleStart;
            this->parentList = parentList;
        }

        virtual const char* what() const throw() {
            return "Graph contains a negative-weight cycle";
        }
    };

    /*
    @brief Perform DFS on the graph
     @param g - the graph to perform DFS on
     @return a list of vertices in the order they were discovered
    */
    static vector<vector<int>> dfs(Graph& g);
    /**
     * @brief Perform DFS from a given source vertex
     * @param g - the graph to perform DFS on
     * @param src - the source vertex to start DFS from
     * @param colors - colors of the vertices
     * @return a list of vertices in the order they were discovered
     */
    static vector<int> dfs(Graph& g, int src, vector<Color>& colors);

    /**
     * @brief Perform BFS from a given source vertex
     * @param g - the graph to perform BFS on
     * @param src - the source vertex to start BFS from
     * @return a pair of two vectors:
     * 1. the first vector contains the distance from the source vertex to each vertex in the graph
     * 2. the second vector contains the parent of each vertex in the graph in the BFS tree
     */
    static pair<vector<int>, vector<int>> bfs(Graph& g, int src);
    /**
     * @brief Perform Bellman-Ford algorithm from a given source vertex
     * @param g - the graph to perform Bellman-Ford algorithm on
     * @param src - the source vertex to start Bellman-Ford algorithm from
     * @return a pair of two vectors:
     * 1. the first vector contains the distance from the source vertex to each vertex in the graph
     * 2. the second vector contains the parent of each vertex in the graph in the BFS tree
     * @throws NegativeCycleException if the graph contains a negative-weight cycle
     */
    static pair<vector<int>, vector<int>> bellmanFord(Graph& g, int src);

    /**
     * @brief Perform Dijkstra's algorithm from a given source vertex
     * @param g - the graph to perform Dijkstra's algorithm on (must be a non-negative weighted graph)
     * @param src - the source vertex to start Dijkstra's algorithm from
     * @return a pair of two vectors:
     * 1. the first vector contains the distance from the source vertex to each vertex in the graph
     * 2. the second vector contains the parent of each vertex in the graph in the BFS tree
     *
     */
    static pair<vector<int>, vector<int>> dijkstra(Graph& g, int src);
};

}  // namespace ariel