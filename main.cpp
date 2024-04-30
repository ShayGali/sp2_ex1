#include <iostream>
#include <vector>

#include "algorithms/Algorithms.hpp"
#include "graph/DirectedGraph.hpp"
#include "graph/Graph.hpp"
#include "graph/UndirectedGraph.hpp"

using namespace std;

int main() {
    UndirectedGraph g;

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 10,     -1},
        {10,     NO_EDGE, -5},
        {-1,     -5,     NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);

    g.loadGraph(graph);
    std::cout << Algorithms::negativeCycle(g) << std::endl;
    return 0;
}