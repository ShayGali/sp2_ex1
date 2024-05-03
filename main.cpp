#include <iostream>
#include <vector>

#include "algorithms/Algorithms.hpp"
#include "graph/Graph.hpp"

using namespace std;

int main() {
    Graph g(false);

    vector<vector<int>> graph = {
        // clang-format off
        //  0          1      2         3        4
    /*0*/{NO_EDGE, 1      , 2      , NO_EDGE, NO_EDGE},
    /*1*/{1      , NO_EDGE, 3      , NO_EDGE, NO_EDGE},
    /*2*/{2      , 3      , NO_EDGE, 4      , NO_EDGE},
    /*3*/{NO_EDGE, NO_EDGE, 4      , NO_EDGE, 5      },
    /*4*/{NO_EDGE, NO_EDGE, NO_EDGE, 5      , NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    std::cout << Algorithms::shortestPath(g, 0, 4) << std::endl;
    return 0;
}