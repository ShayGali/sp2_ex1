#include <iostream>
#include <vector>

#include "algorithms/Algorithms.hpp"
#include "graph/Graph.hpp"

using namespace std;
using namespace shayg;

int main() {
    Graph g(true);

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, -1,      NO_EDGE},
        {1,       NO_EDGE, -5     },
        {NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);
    cout << Algorithms::shortestPath(g, 2, 0) << endl;
    return 0;
}