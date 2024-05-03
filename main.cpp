#include <iostream>
#include <vector>

#include "algorithms/Algorithms.hpp"
#include "graph/Graph.hpp"

using namespace std;

int main() {
    Graph g(true);

    vector<vector<int>> graph = {};
    g.loadGraph(graph);
    
    cout << Algorithms::isConnected(g) << endl;
    return 0;
}