#include <iostream>
#include <vector>

#include "algorithms/Algorithms.hpp"
#include "graph/DirectedGraph.hpp"
#include "graph/Graph.hpp"
#include "graph/UndirectedGraph.hpp"

using namespace std;

int main() {
    DirectedGraph g;
    vector<vector<int>> graph2 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, -1    , -1     , NO_EDGE, NO_EDGE}, 
        /* 1 */{1, NO_EDGE, -1, NO_EDGE, NO_EDGE}, 
        /* 2 */{1, 1, NO_EDGE, -1     , NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, 1, NO_EDGE, -1     }, 
        /* 4 */{NO_EDGE, 1     , NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph2);
    std::cout << Algorithms::shortestPath(g, 0, 1) << std::endl;
    return 0;
}