# System Programming 2 - assignment 1

...


## Graph


## Algorithms

### isConnected

In this function, we will use the DFS algorithm to check if the graph is connected or not.

to check if undirected graph is connected, we can perform DFS on the graph and check if all the vertices are discovered.

The way to check if a directed graph is connected is toperform DFS twice:
1. Perform DFS on the graph.
2. If the DFS discovers all the vertices, then the graph isconnected. (if we got only one DFS tree)
3. Perform DFS on the root of the last DFS tree.
4. If the DFS discovers all the vertices, then the graph isconnected. otherwise, the graph is not connected.

### shortestPath

In this function, we will use the one of 3 algorithms to find the shortest path between two vertices in a graph.

* if the graph is unwieghted, we will use the BFS algorithm to find the shortest path between two vertices.
* if the graph is wieghted and the wieghts are positive, we will use the Dijkstra algorithm to find the shortest path between two vertices.
* if the graph is wieghted and the wieghts are negative, we will use the Bellman-Ford algorithm to find the shortest path between two vertices.

> Note: we represent the graph as an adjacency list, so both Dijkstra and Bellman-Ford algorithms run in $O(V^3)$ time complexity.

## Test