
#include "Algorithms.hpp"

#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <utility>
#include <vector>

enum Color {
    // for DFS
    WHITE,
    GRAY,
    BLACK,

    // for bipartite
    BLUE,
    RED
};

using std::pair, std::vector, std::queue, std::string;

// ~~~ declare the helper functions ~~~

string constructCyclePath(vector<int>& path, int start);

vector<vector<size_t>> dfs(Graph& g);
vector<size_t> dfs(Graph& g, size_t src, vector<Color>* colors);

pair<vector<int>, vector<int>> bfs(Graph& g, size_t src);
pair<vector<int>, vector<int>> bellmanFord(Graph& g, size_t src);
pair<vector<int>, vector<int>> dijkstra(Graph& g, size_t src);

string isContainsCycleUtil(Graph& g, size_t src, vector<Color>* colors, vector<int>* parents, vector<int>* path);

// ~~~ implement the functions ~~~
bool Algorithms::isConnected(Graph& g) {
    /*

    to check if undirected graph is connected, we can perform DFS on the graph and check if all the vertices are discovered.

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

    // if the graph is undirected, the second DFS is not needed
    if (!g.isDirectedGraph()) {
        return false;
    }

    // Perform DFS on the root of the last DFS tree (the first element of the last vector in the firstDfsTree matrix
    size_t firstElementOfLastVector = firstDfsTree.back().front();
    vector<Color> colors(g.getGraph().size(), WHITE);
    vector<size_t> secondDfsTree = dfs(g, firstElementOfLastVector, &colors);
    return secondDfsTree.size() == g.getGraph().size();
}

string Algorithms::shortestPath(Graph& g, size_t src, size_t dest) {
    if (src >= g.getGraph().size() || src < 0 || dest >= g.getGraph().size() || dest < 0) {
        throw std::invalid_argument("Invalid source or destination vertex");
    }

    if (src == dest) {
        return std::to_string(src);
    }

    pair<vector<int>, vector<int>> shortestPathResult;

    // if the graph is not weighted, we can use BFS to find the shortest path
    if (!g.isWeightedGraph()) {
        shortestPathResult = bfs(g, src);
    } else if (g.isHaveNegativeEdgeWeight()) {  // if the graph has negative edge weights, we can use Bellman-Ford algorithm
        try {
            shortestPathResult = bellmanFord(g, src);
        } catch (Algorithms::NegativeCycleException& e) {
            return e.what();
        }
    } else {  // if the graph has non-negative edge weights, we can use Dijkstra's algorithm
        shortestPathResult = dijkstra(g, src);
    }

    // get the shortest path from the src to the dest
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

string Algorithms::isContainsCycle(Graph& g) {
    vector<Color> colors(g.getGraph().size(), WHITE);
    vector<int> parents(g.getGraph().size(), -1);
    vector<int> path;

    for (size_t i = 0; i < g.getGraph().size(); i++) {
        if (colors[i] == WHITE) {
            string cycle = isContainsCycleUtil(g, i, &colors, &parents, &path);
            if (!cycle.empty()) {
                return cycle;
            }
        }
    }
    return "-1";
}

string Algorithms::isBipartite(Graph& g) {
    /*
    To check if a graph is bipartite, we will perform BFS on the graph and color the vertices in two colors.
    If in some point we discover a vertex that is colored with the same color as its parent, then the graph is not bipartite.

    in the end, we will return the two sets of vertices, according to the colors of the vertices.
    */

    size_t n = g.getGraph().size();
    // create a list of colors for the vertices
    vector<Color> colors(n, WHITE);
    // create two sets of vertices
    vector<size_t> setB;
    vector<size_t> setR;

    // start BFS from the first vertex
    vector<size_t> queue;
    queue.push_back(0);
    colors[0] = BLUE;
    setB.push_back(0);

    while (queue.size() != 0) {
        size_t u = queue.back();
        for (size_t v = 0; v < n; v++) {
            if (g.getGraph()[u][v] != NO_EDGE) {
                if (colors[v] == colors[u]) {
                    return "0";  // the graph is not bipartite
                }
                if (colors[v] == WHITE) {
                    if (colors[u] == BLUE) {
                        colors[v] = RED;
                        setR.push_back(v);
                    } else {
                        colors[v] = BLUE;
                        setB.push_back(v);
                    }
                }
                queue.push_back(v);
            }
        }
    }
    // create the result string
    string result = "The graph is bipartite: A={";
    for (size_t i = 0; i < setB.size() - 1; i++) {
        result += std::to_string(setB[i]);
        if (i != setB.size() - 1) {
            result += ", ";
        }
    }
    result += std::to_string(setB.back()) + "}, B={";
    for (size_t i = 0; i < setR.size() - 1; i++) {
        result += std::to_string(setR[i]);
        if (i != setR.size() - 1) {
            result += ", ";
        }
    }
    result += std::to_string(setR.back()) + "}";
    return result;
}

string Algorithms::negativeCycle(Graph& g) {
    /*
    To find a negative cycle in the graph, we will add a new vertex to the graph and connect it to all the other vertices with an edge of weight 0.
    Then we will perform Bellman-Ford algorithm from the new vertex.

    on the last part on the Bellman-Ford algorithm, if we can relax an edge, then the graph contains a negative cycle.

    to get the negative cycle, we will go back with the `parents` vector until we reach the vertex we started from.

    because in our graph representation, no edges have a weight of 0, in this function we will change the NO_EDGE value to INFINITY.
    */

    size_t n = g.getGraph().size();

    // create a new graph with a new vertex
    Graph newGraph = Graph(g.isDirectedGraph());

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
        newGraphMat[i][n] = 0;  // //TODO: check if we need to be NO_EDGE
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

// ~~~ implement the helper functions ~~~

// ~ DFS ~

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

// ~ Shortest Path algorithms ~
pair<vector<int>, vector<int>> bfs(Graph& g, size_t src) {
    size_t n = g.getGraph().size();

    // init the distances and parents vectors
    vector<int> distances(n, INF);
    vector<int> parents(n, -1);

    // add the source vertex to the queue and set its distance to 0
    distances[src] = 0;
    std::queue<size_t> q;
    q.push(src);

    while (!q.empty()) {
        size_t u = q.front();
        q.pop();
        for (size_t v = 0; v < n; v++) {
            if (g.getGraph()[u][v] != NO_EDGE) {
                if (distances[v] == INF) {  // if the vertex is not discovered yet
                    distances[v] = distances[u] + 1;
                    parents[v] = (int)u;
                    q.push(v);
                }
            }
        }
    }

    return {distances, parents};
}
pair<vector<int>, vector<int>> bellmanFord(Graph& g, size_t src) {
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
                    // if the graph is undirected, we should ignore the edge that connects the current vertex to its parent
                    if (!g.isDirectedGraph() && parents[u] == (int)v) {
                        continue;
                    }

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

// ~ Cycle detection ~

/**
 * @brief Check if the graph contains a cycle
 * @param g - the graph to check
 * @param src - the source vertex to start the DFS from
 * @param colors - colors of the vertices
 * @param parents - the parent of each vertex in the DFS tree
 * @param path - a vector to store sequence of vertices visited in during the DFS.
 * @return the cycle path in the format "v1->v2->...->v1" if a cycle is detected, otherwise return an empty string
 */
string isContainsCycleUtil(Graph& g, size_t src, vector<Color>* colors, vector<int>* parents, vector<int>* path) {
    /*
    the path vector is used to store the sequence of vertices visited during the DFS.
    when  a vertex is visited, it is added to the path.
    when all the adjacent vertices of a vertex have been visited, the vertex is removed from the path.
    if a cycle is detected, the path at that point in time represents the cycle.
    the end of the cycle is the vertex that is being visited when the cycle is detected. (start of the cycle is the same as the end of the cycle)
    */

    (*colors)[src] = GRAY;
    path->push_back(src);  // add the vertex to the path

    // loop over all the neighbors of the vertex
    for (size_t v = 0; v < g.getGraph().size(); v++) {
        if (g.getGraph()[src][v] != NO_EDGE) {
            if ((*colors)[v] == WHITE) {
                // dfs on the neighbor
                (*parents)[v] = (int)src;
                string cycle = isContainsCycleUtil(g, v, colors, parents, path);
                if (!cycle.empty()) {  // if a cycle is detected
                    return cycle;
                }
            } else if ((*colors)[v] == GRAY) {  // if the neighbor is gray, then we have a cycle
                // if the graph is undirected, we should ignore the edge that connects the current vertex to its parent
                if (!g.isDirectedGraph() && (*parents)[src] == (int)v) {
                    continue;
                }
                // construct the cycle path
                return constructCyclePath(*path, v);
            }
        }
    }
    (*colors)[src] = BLACK;  // the vertex is done, mark it as black
    path->pop_back();        // remove the vertex from the path because we are done with it
    return "";
}

/**
 * @brief Build the cycle path from the path vector
 * @param path - the path vector - used to store the sequence of vertices visited during the DFS when a cycle is detected
 * @param start - the start/end of the cycle (start == end)
 * @return the cycle path in the format "v1->v2->...->v1"
 */
string constructCyclePath(vector<int>& path, int start) {
    string cycle = "";
    size_t v = 0;
    // find the start of the cycle in the path
    for (v = 0; v < path.size(); v++) {
        if (path[v] == start) {
            break;
        }
    }
    // we start from the start of the cycle, until the end of the path (when the cycle is detected)
    for (size_t i = v; i < path.size(); i++) {
        cycle += std::to_string(path[i]) + "->";
    }

    cycle += std::to_string(start);  // to complete the cycle
    return cycle;
}