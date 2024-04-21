#include <iostream>
#include <vector>

#include "algorithms/Algorithms.hpp"
#include "graph/DirectedGraph.hpp"
#include "graph/Graph.hpp"
#include "graph/UndirectedGraph.hpp"

using namespace std;

int main() {
    DirectedGraph g;
     vector<vector<int>> graph3 = {
        // clang-format off
        {NO_EDGE, -1,      NO_EDGE},
        {NO_EDGE, NO_EDGE, -1     },
        {1,       NO_EDGE, NO_EDGE}
    };

    g.loadGraph(graph3);
    std::cout << Algorithms::negativeCycle(g) << std::endl;
    return 0;
}