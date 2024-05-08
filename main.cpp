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
    { NO_EDGE, 5      , 3      , NO_EDGE, NO_EDGE }, // A
    { NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE }, // B
    { NO_EDGE, NO_EDGE, NO_EDGE, -2     , NO_EDGE }, // C
    { NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, -1      }, // D
    { NO_EDGE, NO_EDGE, -1     , NO_EDGE, NO_EDGE }  // E
        // clang-format on
    };

    g.loadGraph(graph);
    cout << Algorithms::negativeCycle(g) << endl;
    return 0;
}