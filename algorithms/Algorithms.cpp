
#include "Algorithms.hpp"

#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <utility>
#include <vector>

using std::pair;

// declare the helper functions
bool isContainsCycleUtil(UndirectedGraph& g, size_t src, vector<Color>* colors, vector<int>* parents);
bool isContainsCycleUtil(DirectedGraph& g, size_t src, vector<Color>* colors);
vector<vector<size_t>> dfs(Graph& g);
vector<size_t> dfs(Graph& g, size_t src, vector<Color>* colors);
pair<vector<int>, vector<int>> bellmanFord(UndirectedGraph& g, size_t src);
pair<vector<int>, vector<int>> bellmanFord(DirectedGraph& g, size_t src);
pair<vector<int>, vector<int>> dijkstra(Graph& g, size_t src);

bool Algorithms::isConnected(DirectedGraph& g) {
    /*
   The way to check if a directed graph is connected is to perform DFS twice:

   1. Perform DFS on the graph.
   2. If the DFS discovers all the vertices, then the graph is connected. (if we got only one DFS tree)
   3. Perform DFS on the root of the last DFS tree.
   4. If the DFS discovers all the vertices, then the graph is connected.
   */

    vector<vector<size_t>> firstDfsTree = dfs(g);
    if (firstDfsTree.size() == 1) {
        return true;
    }
    // Perform DFS on the root of the last DFS tree (the first element of the last vector in the firstDfsTree matrix
    size_t firstElementOfLastVector = firstDfsTree.back().front();
    vector<Color> colors(g.getGraph().size(), WHITE);
    vector<size_t> secondDfsTree = dfs(g, firstElementOfLastVector, &colors);
    return secondDfsTree.size() == g.getGraph().size();
    return false;
}

bool Algorithms::isConnected(UndirectedGraph& g) {
    /*
   The way to check if an undirected graph is connected is to perform DFS on the graph:

   1. Perform DFS on the graph.
   2. If the DFS discovers all the vertices, then the graph is connected.
   */

    vector<vector<size_t>> dfsTree = dfs(g);

    return dfsTree.size() == 1;
}

string Algorithms::shortestPath(DirectedGraph& g, size_t src, size_t dest) {
    if (src == dest) {
        return std::to_string(src);
    }

    pair<vector<int>, vector<int>> shortestPathResult;

    if (g.isHaveNegativeEdgeWeight()) {
        // do Bellman-Ford and return the path
        try {
            shortestPathResult = bellmanFord(g, src);
        } catch (Algorithms::NegativeCycleException e) {
            return e.what();
        }
    } else {
        // do Dijkstra and return the path
        shortestPathResult = dijkstra(g, src);
    }
    vector<int> distances = shortestPathResult.first;
    vector<int> parents = shortestPathResult.second;
    // if the distance to the destination vertex is infinity, then there is no path between the source and destination vertices
    if (distances[dest] == INF) {
        return "-1";
    }

    // create the path from the source to the destination
    string path = std::to_string(dest);
    int parent = parents[dest];
    while (parent != -1) {
        path = std::to_string(parent) + "->" + path;
        parent = parents[(size_t)parent];
    }
    return path;
}

string Algorithms::shortestPath(UndirectedGraph& g, size_t src, size_t dest) {
    if (src == dest) {
        return std::to_string(src);
    }

    pair<vector<int>, vector<int>> shortestPathResult;

    if (g.isHaveNegativeEdgeWeight()) {
        // do Bellman-Ford and return the path
        try {
            shortestPathResult = bellmanFord(g, src);
        } catch (Algorithms::NegativeCycleException e) {
            return e.what();
        }
    } else {
        // do Dijkstra and return the path
        shortestPathResult = dijkstra(g, src);
    }
    vector<int> distances = shortestPathResult.first;
    vector<int> parents = shortestPathResult.second;
    // if the distance to the destination vertex is infinity, then there is no path between the source and destination vertices
    if (distances[dest] == INF) {
        return "-1";
    }

    // create the path from the source to the destination
    string path = std::to_string(dest);
    int parent = parents[dest];
    while (parent != -1) {
        path = std::to_string(parent) + "->" + path;
        parent = parents[(size_t)parent];
    }
    return path;
}

bool Algorithms::isContainsCycle(UndirectedGraph& g) {
    /*
    To check if the graph contains a cycle, we can perform DFS on the graph and check if there is a back edge in the graph.
    */
    vector<Color> colors(g.getGraph().size(), WHITE);
    vector<int> parents(g.getGraph().size(), -1);

    for (size_t i = 0; i < g.getGraph().size(); i++) {
        if (colors[i] == WHITE) {
            if (isContainsCycleUtil(g, i, &colors, &parents)) {
                return true;
            }
        }
    }
    return false;
}

bool isContainsCycleUtil(UndirectedGraph& g, size_t src, vector<Color>* colors, vector<int>* parents) {
    (*colors)[src] = GRAY;
    for (size_t v = 0; v < g.getGraph().size(); v++) {
        if (g.getGraph()[src][v] != NO_EDGE) {
            if ((*colors)[v] == WHITE) {
                (*parents)[v] = (int)src;
                if (isContainsCycleUtil(g, v, colors, parents)) {
                    return true;
                }
            } else if ((*colors)[v] == GRAY && v != (*parents)[src]) {
                return true;
            }
        }
    }
    (*colors)[src] = BLACK;
    return false;
}

bool Algorithms::isContainsCycle(DirectedGraph& g) {
    /*
    To check if the graph contains a cycle, we can perform DFS on the graph and check if there is a back edge in the graph.
    */
    vector<Color> colors(g.getGraph().size(), WHITE);
    for (size_t i = 0; i < g.getGraph().size(); i++) {
        if (colors[i] == WHITE) {
            if (isContainsCycleUtil(g, i, &colors)) {
                return true;
            }
        }
    }
    return false;
}

bool isContainsCycleUtil(DirectedGraph& g, size_t src, vector<Color>* colors) {
    (*colors)[src] = GRAY;
    for (size_t v = 0; v < g.getGraph().size(); v++) {
        if (g.getGraph()[src][v] != NO_EDGE) {
            if ((*colors)[v] == GRAY) {
                return true;
            } else if ((*colors)[v] == WHITE) {
                if (isContainsCycleUtil(g, v, colors)) {
                    return true;
                }
            }
        }
    }
    (*colors)[src] = BLACK;
    return false;
}

string Algorithms::isBipartite(Graph& g) {
    /*
    To check if a graph is bipartite, we will perform DFS on the graph and color the vertices in two colors.
    If in some point we discover a vertex that is colored with the same color as its parent, then the graph is not bipartite.

    in the end, we will return the two sets of vertices, according to the colors of the vertices.
    */

    size_t n = g.getGraph().size();
    // create a list of colors for the vertices
    vector<Color> colors(n, WHITE);

    // create a stack to store the vertices (instead of recursion)
    vector<size_t> stack;

    // create two sets of vertices
    vector<size_t> setA;
    vector<size_t> setB;

    // start loop over all vertices
    for (size_t i = 0; i < n; i++) {
        if (colors[i] == WHITE) {
            // do DFS from vertex i
            stack.push_back(i);
            colors[i] = RED;
            setA.push_back(i);
            while (!stack.empty()) {
                // get the last vertex from the stack
                size_t u = stack.back();
                stack.pop_back();
                // check the color of the vertex
                if (colors[u] == RED) {
                    // loop over the neighbors of the vertex and color them
                    for (size_t v = 0; v < n; v++) {
                        if (g.getGraph()[u][v] != NO_EDGE) {
                            // if there is an edge between u and v, and v not discovered yet
                            if (colors[v] == WHITE) {
                                colors[v] = BLUE;
                                setB.push_back(v);
                                stack.push_back(v);
                            } else if (colors[v] == RED) {  // if v is discovered and colored with the same color as u the graph is not bipartite
                                return "0";
                            }
                        }
                    }
                } else if (colors[u] == BLUE) {
                    for (size_t v = 0; v < n; v++) {
                        if (g.getGraph()[u][v] != NO_EDGE) {
                            if (colors[v] == WHITE) {
                                colors[v] = RED;
                                setA.push_back(v);
                                stack.push_back(v);
                            } else if (colors[v] == BLUE) {
                                return "0";
                            }
                        }
                    }
                }
            }
        }
    }

    // create the result string
    string result = "The graph is bipartite: A={";
    for (size_t i = 0; i < setA.size() - 1; i++) {
        result += std::to_string(setA[i]);
        if (i != setA.size() - 1) {
            result += ", ";
        }
    }
    result += std::to_string(setA.back()) + "}, B={";

    for (size_t i = 0; i < setB.size() - 1; i++) {
        result += std::to_string(setB[i]);
        if (i != setB.size() - 1) {
            result += ", ";
        }
    }

    result += std::to_string(setB.back()) + "}";
    return result;
}

string Algorithms::negativeCycle(DirectedGraph& g) {
    /*
    To find a negative cycle in the graph, we will add a new vertex to the graph and connect it to all the other vertices with an edge of weight 0.
    Then we will perform Bellman-Ford algorithm from the new vertex.

    on the last part on the Bellman-Ford algorithm, if we can relax an edge, then the graph contains a negative cycle.

    to get the negative cycle, we will go back with the `parents` vector until we reach the vertex we started from.

    because in our graph representation, no edges have a weight of 0, in this function we will change the NO_EDGE value to INFINITY.
    */

    size_t n = g.getGraph().size();

    // create a new graph with a new vertex
    DirectedGraph newGraph = DirectedGraph();
    vector<vector<int>> newGraphMat(n + 1, vector<int>(n + 1, INF));
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++) {
            if (g.getGraph()[i][j] != NO_EDGE) {
                newGraphMat[i][j] = g.getGraph()[i][j];
            }
        }
    }

    // connect the new vertex to all the other vertices with an edge of weight 0
    for (size_t i = 0; i < n; i++) {
        newGraphMat[n][i] = 0;
    }

    newGraph.loadGraph(newGraphMat);

    // start Bellman-Ford algorithm from the new vertex
    try {
        bellmanFord(newGraph, n);
    } catch (Algorithms::NegativeCycleException e) {  // if the graph contains a negative cycle
        // get the negative cycle
        string negativeCycle = std::to_string(e.detectedCycleStart);
        size_t parent = e.detectedCycleStart;
        while (e.parentList[parent] != e.detectedCycleStart) {
            parent = (size_t)e.parentList[parent];
            negativeCycle += "->" + std::to_string(parent);
        }
        negativeCycle += "->" + std::to_string(e.detectedCycleStart);
        return negativeCycle;
    }
    return "No negative cycle";
}

string Algorithms::negativeCycle(UndirectedGraph& g) {
    size_t n = g.getGraph().size();

    // create a new graph with a new vertex
    UndirectedGraph newGraph = UndirectedGraph();
    vector<vector<int>> newGraphMat(n + 1, vector<int>(n + 1, INF));
    for (size_t i = 0; i < n; i++) {
        for (size_t j = i + 1; j < n; j++) {
            if (g.getGraph()[i][j] != NO_EDGE) {
                newGraphMat[i][j] = g.getGraph()[i][j];
                newGraphMat[j][i] = g.getGraph()[i][j];
            }
        }
    }

    // connect the new vertex to all the other vertices with an edge of weight 0
    for (size_t i = 0; i < n; i++) {
        newGraphMat[n][i] = 0;
        newGraphMat[i][n] = 0;
    }

    newGraph.loadGraph(newGraphMat);

    // start Bellman-Ford algorithm from the new vertex
    try {
        bellmanFord(newGraph, n);
    } catch (Algorithms::NegativeCycleException e) {  // if the graph contains a negative cycle
        // get the negative cycle
        size_t cycleStart = e.detectedCycleStart;
        int cycleEnd = e.parentList[(size_t)cycleStart];
        size_t maxIterations = n + 1;
        while (cycleEnd != cycleStart && maxIterations > 0) {
            cycleStart = (size_t)cycleEnd;
            cycleEnd = e.parentList[cycleStart];
            maxIterations--;
        }

        if (maxIterations == 0) {
            return "Error: Cycle detection failed";
        }

        string negativeCycle = std::to_string(cycleStart);
        int parent = e.parentList[cycleStart];
        maxIterations = n + 1;
        while (parent != cycleStart && maxIterations > 0) {
            negativeCycle += "->" + std::to_string(parent);
            parent = e.parentList[(size_t)parent];
            maxIterations--;
        }

        if (maxIterations == 0) {
            return "Error: Cycle detection failed";
        }

        negativeCycle += "->" + std::to_string(cycleStart);
        return negativeCycle;
    }
    return "No negative cycle";
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// helper functions

/**
  @brief Perform DFS on the graph
  @param g - the graph to perform DFS on
  @return a list of vertices in the order they were discovered
*/
vector<vector<size_t>> dfs(Graph& g) {
    size_t n = g.getGraph().size();
    // create a list of colors for the vertices
    vector<Color> colors(n, WHITE);

    // create result dfs "tree" (we will return it as a matrix of edges)
    vector<vector<size_t>> dfsTree;

    // start loop over all vertices
    for (size_t i = 0; i < n; i++) {
        if (colors[i] == WHITE) {
            // do DFS from vertex i
            dfsTree.push_back(dfs(g, i, &colors));
        }
    }
    return dfsTree;
}
/**
 * @brief Perform DFS from a given source vertex
 * @param g - the graph to perform DFS on
 * @param src - the source vertex to start DFS from
 * @param colors - colors of the vertices
 * @return a list of vertices in the order they were discovered
 */
vector<size_t> dfs(Graph& g, size_t src, vector<Color>* colors) {
    size_t n = g.getGraph().size();

    vector<size_t> dfsOrder;

    // create a stack to store the vertices (instead of recursion)
    vector<size_t> stack;

    stack.push_back(src);
    while (!stack.empty()) {
        // get the last vertex from the stack
        size_t u = stack.back();
        stack.pop_back();
        if ((*colors)[u] == WHITE) {  // if the vertex is white - we just discovered it
            // discover the vertex and loop over its neighbors
            (*colors)[u] = GRAY;
            dfsOrder.push_back(u);
            for (size_t v = 0; v < n; v++) {
                if (g.getGraph()[u][v] != NO_EDGE) {  // if there is an edge between u and v
                    if ((*colors)[v] == WHITE) {      // if we didn't discover v yet
                        stack.push_back(v);           // add v to the stack
                    }
                }
            }
        } else if ((*colors)[u] == GRAY) {
            (*colors)[u] = BLACK;
        }
    }

    return dfsOrder;
};

/**
 * @brief Perform Bellman-Ford algorithm from a given source vertex
 * @param g - the graph to perform Bellman-Ford algorithm on
 * @param src - the source vertex to start Bellman-Ford algorithm from
 * @return a pair of two vectors:
 * 1. the first vector contains the distance from the source vertex to each vertex in the graph
 * 2. the second vector contains the parent of each vertex in the graph in the BFS tree
 * @throws NegativeCycleException if the graph contains a negative-weight cycle
 */
pair<vector<int>, vector<int>> bellmanFord(UndirectedGraph& g, size_t src) {
    /*
    we relax the eged twice, once from u to v and once from v to u
    */
    size_t n = g.getGraph().size();
    vector<int> distances(n, INF);
    vector<int> parents(n, -1);

    distances[src] = 0;
    // relax all edges n-1 times
    for (size_t i = 0; i < n - 1; i++) {
        // for each edge (u, v) in the graph
        for (size_t u = 0; u < n; u++) {
            for (size_t v = u + 1; v < n; v++) {
                // if there is an edge between u and v
                if (g.getGraph()[u][v] != NO_EDGE) {
                    // relax the edge
                    if (distances[u] + g.getGraph()[u][v] < distances[v]) {
                        distances[v] = distances[u] + g.getGraph()[u][v];
                        parents[v] = (int)u;
                    }
                    if (distances[v] + g.getGraph()[u][v] < distances[u]) {
                        distances[u] = distances[v] + g.getGraph()[u][v];
                        parents[u] = (int)v;
                    }
                }
            }
        }
    }

    // check for negative-weight cycles
    for (size_t u = 0; u < n; u++) {
        for (size_t v = u + 1; v < n; v++) {
            if (g.getGraph()[u][v] != NO_EDGE) {
                if (distances[u] + g.getGraph()[u][v] < distances[v] || distances[v] + g.getGraph()[u][v] < distances[u]) {
                    throw Algorithms::NegativeCycleException(u, parents);
                }
            }
        }
    }

    return std::make_pair(distances, parents);
}

pair<vector<int>, vector<int>> bellmanFord(DirectedGraph& g, size_t src) {
    size_t n = g.getGraph().size();
    vector<int> distances(n, INF);
    vector<int> parents(n, -1);

    distances[src] = 0;
    // relax all edges n-1 times
    for (size_t i = 0; i < n - 1; i++) {
        // for each edge (u, v) in the graph
        for (size_t u = 0; u < n; u++) {
            for (size_t v = 0; v < n; v++) {
                // if there is an edge between u and v
                if (g.getGraph()[u][v] != NO_EDGE) {
                    // relax the edge
                    if (distances[u] + g.getGraph()[u][v] < distances[v]) {
                        distances[v] = distances[u] + g.getGraph()[u][v];
                        parents[v] = (int)u;
                    }
                }
            }
        }
    }

    // check for negative-weight cycles
    for (size_t u = 0; u < n; u++) {
        for (size_t v = 0; v < n; v++) {
            if (g.getGraph()[u][v] != NO_EDGE) {
                if (distances[u] + g.getGraph()[u][v] < distances[v]) {
                    throw Algorithms::NegativeCycleException(u, parents);
                }
            }
        }
    }

    return std::make_pair(distances, parents);
}

/**
 * @brief Perform Dijkstra's algorithm from a given source vertex
 * @param g - the graph to perform Dijkstra's algorithm on (must be a non-negative weighted graph)
 * @param src - the source vertex to start Dijkstra's algorithm from
 * @return a pair of two vectors:
 * 1. the first vector contains the distance from the source vertex to each vertex in the graph
 * 2. the second vector contains the parent of each vertex in the graph in the BFS tree
 *
 */
pair<vector<int>, vector<int>> dijkstra(Graph& g, size_t src) {
    size_t n = g.getGraph().size();
    vector<int> distances(n, INF);
    vector<int> parents(n, -1);

    // create priority queue - min heap
    /* in here:
    1. pair<int, size_t> - first int is the distance from the source vertex to the vertex, second int is the vertex
    2. vector<pair<int, size_t>> - the type of the container (what is the container that holds the elements)
    3. greater<pair<int, int>> - the comparator (how to compare the elements in the container)
        greater is a functor that compares two elements and returns true if the first element is greater than the second element
        when we pass pair<int, int> to the priority_queue, it will compare the first element of the pair
    */
    std::priority_queue<pair<int, size_t>, vector<pair<int, size_t>>, std::greater<pair<int, int>>> pq;

    // initialize source vertex
    distances[src] = 0;
    pq.push(std::make_pair(0, src));

    while (!pq.empty()) {
        // get the vertex with the smallest distance
        size_t u = pq.top().second;
        pq.pop();

        // for each neighbor of u
        for (size_t v = 0; v < n; v++) {
            if (g.getGraph()[u][v] != NO_EDGE) {
                // relax the edge
                int currDist = distances[u] + g.getGraph()[u][v];
                if (currDist < distances[v]) {
                    distances[v] = currDist;
                    parents[v] = u;
                    pq.push(std::make_pair(distances[v], v));
                }
            }
        }
    }

    return make_pair(distances, parents);
}