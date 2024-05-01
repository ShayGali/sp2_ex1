#include <iostream>
#include <stdexcept>
#include <vector>

#include "../algorithms/Algorithms.hpp"
#include "../graph/Graph.hpp"
#include "doctest.h"

using namespace std;

TEST_CASE("Test loadGraph for Directed Graph") {
    Graph g = Graph(true);
    vector<vector<int>> graph = {
        // clang-format off
            {NO_EDGE, 1,       1      },
            {1,       NO_EDGE, 1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    vector<vector<int>> ajdList = g.getGraph();
    CHECK(std::equal(graph.begin(), graph.end(), ajdList.begin()));

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the graph is not a square matrix
    vector<vector<int>> graph2 = {
        // clang-format off
            {NO_EDGE, 1,       1},
            {1,       NO_EDGE, 1}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph2), invalid_argument);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the diagonal of the matrix is not NO_EDGE
    vector<vector<int>> graph3 = {
        // clang-format off
            {NO_EDGE, 1,       1      },
            {1,       1,       1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph3), invalid_argument);
}

TEST_CASE("Test loadGraph for UndirectedGraph") {
    Graph g = Graph(false);
    vector<vector<int>> graph = {
        // clang-format off
            {NO_EDGE, 1,       1      },
            {1,       NO_EDGE, 1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    vector<vector<int>> ajdList = g.getGraph();
    CHECK(std::equal(graph.begin(), graph.end(), ajdList.begin()));

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the graph is not a square matrix
    vector<vector<int>> graph2 = {
        // clang-format off
            {NO_EDGE, 1,       1},
            {1,       NO_EDGE, 1}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph2), invalid_argument);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the diagonal of the matrix is not NO_EDGE
    vector<vector<int>> graph3 = {
        // clang-format off
            {NO_EDGE, 1,       1      },
            {1,       1,       1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph3), invalid_argument);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // non-symmetric matrix will cause an exception
    vector<vector<int>> graph4 = {
        // clang-format off
            {NO_EDGE, 2,       1      },
            {1,       NO_EDGE, 1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph4), invalid_argument);
}

TEST_CASE("Test isConnected for DirectedGraph") {
    Graph g = Graph(true);
    /*
    0-->1-->2
    */
    vector<vector<int>> graph = {
        // clang-format off
            {NO_EDGE, 1,       NO_EDGE},
            {NO_EDGE, NO_EDGE, 1      },
            {NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    CHECK(Algorithms::isConnected(g) == true);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the 5th vertex is not connected to any other vertex
    vector<vector<int>> graph2 = {
        // clang-format off
            {NO_EDGE, 1,       1,       NO_EDGE, NO_EDGE},
            {1,       NO_EDGE, 1,       NO_EDGE, NO_EDGE},
            {1,       1,       NO_EDGE, 1,       NO_EDGE},
            {NO_EDGE, NO_EDGE, 1,       NO_EDGE, NO_EDGE},
            {NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph2);
    CHECK(Algorithms::isConnected(g) == false);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // graph with 1 vertex
    vector<vector<int>> graph3 = {{NO_EDGE}};
    g.loadGraph(graph3);
    CHECK(Algorithms::isConnected(g) == true);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the root of the graph is not the first vertex
    vector<vector<int>> graph4 = {
        // clang-format off
            {NO_EDGE, 1,       NO_EDGE, NO_EDGE},
            {NO_EDGE, NO_EDGE, 1,       NO_EDGE},
            {1,       NO_EDGE, NO_EDGE, NO_EDGE},
            {NO_EDGE, 1,       NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph4);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected for UndirectedGraph") {
    Graph g = Graph(false);
    /*
    0---1---2
    */
    vector<vector<int>> graph = {
        // clang-format off
            {NO_EDGE, 1,       NO_EDGE},
            {1,       NO_EDGE, 1      },
            {NO_EDGE, 1,       NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    CHECK(Algorithms::isConnected(g) == true);

    // the 5th vertex is not connected to any other vertex
    vector<vector<int>> graph2 = {
        // clang-format off
        {NO_EDGE, 1,       1,       NO_EDGE, NO_EDGE},
        {1,       NO_EDGE, 1,       NO_EDGE, NO_EDGE},
        {1,       1,       NO_EDGE, 1,       NO_EDGE},
        {NO_EDGE, NO_EDGE, 1,       NO_EDGE, NO_EDGE},
        {NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph2);
    CHECK(Algorithms::isConnected(g) == false);

    // graph with 1 vertex
    vector<vector<int>> graph3 = {{NO_EDGE}};
    g.loadGraph(graph3);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test shortestPath for DirectedGraph unweighted") {
    Graph g = Graph(true);
    /*
    0-->1-->2
    */
    vector<vector<int>> graph = {
        // clang-format off
            {NO_EDGE, 1,       NO_EDGE},
            {NO_EDGE, NO_EDGE, 1      },
            {NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    CHECK(Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // there is no path between the source and destination vertices
    vector<vector<int>> graph2 = {
        // clang-format off
                    // 0      1        2        3        4
            /* 0 */{NO_EDGE, 1,       1,       NO_EDGE, NO_EDGE}, 
            /* 1 */{1,       NO_EDGE, 1,       NO_EDGE, NO_EDGE}, 
            /* 2 */{1,       1,       NO_EDGE, 1,       NO_EDGE}, 
            /* 3 */{NO_EDGE, NO_EDGE, 1,       NO_EDGE, 1      }, 
            /* 4 */{NO_EDGE, NO_EDGE, NO_EDGE, 1,       NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph2);
    CHECK(Algorithms::shortestPath(g, 0, 4) == "0->2->3->4");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the source and destination vertices are the same
    CHECK(Algorithms::shortestPath(g, 0, 0) == "0");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // there is no path between the source and destination vertices
    vector<vector<int>> graph3 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 1 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 2 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE }, 
        /* 4 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph3);
    CHECK(Algorithms::shortestPath(g, 0, 4) == "-1");
    CHECK(Algorithms::shortestPath(g, 2, 3) == "-1");
}

TEST_CASE("Test shortestPath for DirectedGraph weighted non-negative") {
    Graph g = Graph(true);

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 3      , NO_EDGE},
        {1      , NO_EDGE, 5      },
        {NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");
    CHECK(Algorithms::shortestPath(g, 0, 2) == "0->1->2");
    CHECK(Algorithms::shortestPath(g, 1, 2) == "1->2");
    CHECK(Algorithms::shortestPath(g, 2, 0) == "-1");
    CHECK(Algorithms::shortestPath(g, 0, 0) == "0");

    vector<vector<int>> graph1 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, 100    , 30     , NO_EDGE, NO_EDGE}, 
        /* 1 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 2 */{NO_EDGE, NO_EDGE, NO_EDGE, 25     , NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, 50     }, 
        /* 4 */{NO_EDGE, 21     , NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");

    graph1[3][4] = 1;
    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->2->3->4->1");
}

TEST_CASE("Test shortestPath for DirectedGraph weighted with negative weights") {
    Graph g = Graph(true);

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, -1,      NO_EDGE},
        {1,       NO_EDGE, -5     },
        {NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");
    CHECK(Algorithms::shortestPath(g, 0, 2) == "0->1->2");
    CHECK(Algorithms::shortestPath(g, 1, 2) == "1->2");
    CHECK(Algorithms::shortestPath(g, 2, 0) == "-1");
    CHECK(Algorithms::shortestPath(g, 0, 0) == "0");

    vector<vector<int>> graph1 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, -100    , -30     , NO_EDGE, NO_EDGE}, 
        /* 1 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 2 */{NO_EDGE, NO_EDGE, NO_EDGE, -25     , NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, -50     }, 
        /* 4 */{NO_EDGE, -21     , NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->2->3->4->1");

    graph1[0][1] = -1000;
    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // graph that dijkstra's algorithm cannot solve
    vector<vector<int>> graph2 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, 0      , NO_EDGE, 2      , -2     }, 
        /* 1 */{NO_EDGE, NO_EDGE, -1     , NO_EDGE, NO_EDGE}, 
        /* 2 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, -8     }, 
        /* 4 */{NO_EDGE, NO_EDGE, 2      , NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph2);
    CHECK(Algorithms::shortestPath(g, 0, 2) == "0->3->4->2");  // in dijkstra's algorithm, the path is 0->1->2

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // negative cycle
    vector<vector<int>> graph3 = {
        // clang-format off
        {NO_EDGE, -1,      NO_EDGE},
        {NO_EDGE, NO_EDGE, -1     },
        {1,       NO_EDGE, NO_EDGE}
    };
    g.loadGraph(graph3);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "Graph contains a negative-weight cycle");
}

TEST_CASE("Test shortestPath for UndirectedGraph unweighted") {
    Graph g = Graph(false);
    /*
    0---1---2
    */
    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 1,       NO_EDGE},
        {1,       NO_EDGE, 1      },
        {NO_EDGE, 1,       NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");
    CHECK(Algorithms::shortestPath(g, 0, 2) == "0->1->2");
    CHECK(Algorithms::shortestPath(g, 1, 2) == "1->2");
    CHECK(Algorithms::shortestPath(g, 2, 0) == "2->1->0");
    CHECK(Algorithms::shortestPath(g, 0, 0) == "0");

    vector<vector<int>> graph1 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, 1,       1,       NO_EDGE, NO_EDGE}, 
        /* 1 */{1,       NO_EDGE, 1,       NO_EDGE, NO_EDGE}, 
        /* 2 */{1,       1,       NO_EDGE, 1,       NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, 1,       NO_EDGE, 1      }, 
        /* 4 */{NO_EDGE, NO_EDGE, NO_EDGE, 1,       NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");
    CHECK(Algorithms::shortestPath(g, 0, 2) == "0->2");
    CHECK(Algorithms::shortestPath(g, 0, 4) == "0->2->3->4");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // two ways to reach the destination vertex

    vector<vector<int>> graph2 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, 1      , 1      , NO_EDGE, NO_EDGE}, 
        /* 1 */{1      , NO_EDGE, 1      , 1      , NO_EDGE}, 
        /* 2 */{1      , 1      , NO_EDGE, 1      , NO_EDGE}, 
        /* 3 */{NO_EDGE, 1      , 1      , NO_EDGE, 1      }, 
        /* 4 */{NO_EDGE, NO_EDGE, NO_EDGE, 1      , NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph2);
    std::string res = Algorithms::shortestPath(g, 0, 4);
    CHECK((res == "0->1->3->4" || res == "0->2->3->4"));

    // no path between the source and destination vertices
    vector<vector<int>> graph3 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 1 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 2 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}, 
        /* 4 */{NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph3);
    CHECK(Algorithms::shortestPath(g, 0, 4) == "-1");
    CHECK(Algorithms::shortestPath(g, 2, 3) == "-1");
    CHECK(Algorithms::shortestPath(g, 0, 0) == "0");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vector<vector<int>> graph4 = {
        //clang-format off
        /* 0 */ {NO_EDGE, 1, NO_EDGE},
        /* 1 */ {1, NO_EDGE, NO_EDGE},
        /* 2 */ {NO_EDGE, NO_EDGE, NO_EDGE}
        //clang-format on
    };

    g.loadGraph(graph4);
    CHECK(Algorithms::shortestPath(g, 0, 2) == "-1");
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");
    CHECK(Algorithms::shortestPath(g, 1, 2) == "-1");
    CHECK(Algorithms::shortestPath(g, 1, 0) == "1->0");
};

TEST_CASE("Test shortestPath for UndirectedGraph weighted non-negative") {
    Graph g = Graph(false);

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 3      , NO_EDGE},
        {3      , NO_EDGE, 5},
        {NO_EDGE, 5      , NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");
    CHECK(Algorithms::shortestPath(g, 0, 2) == "0->1->2");
    CHECK(Algorithms::shortestPath(g, 1, 2) == "1->2");
    CHECK(Algorithms::shortestPath(g, 2, 0) == "2->1->0");
    CHECK(Algorithms::shortestPath(g, 0, 0) == "0");

    vector<vector<int>> graph1 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, 100    , 30     , NO_EDGE, NO_EDGE}, 
        /* 1 */{100,    NO_EDGE, 1,       NO_EDGE, NO_EDGE}, 
        /* 2 */{30,     1,       NO_EDGE, 25     , NO_EDGE}, 
        /* 3 */{NO_EDGE, NO_EDGE, 25,      NO_EDGE, 50     }, 
        /* 4 */{NO_EDGE, NO_EDGE, NO_EDGE, 50     , NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->2->1");

    graph1[1][2] = 1000;
    graph1[2][1] = 1000;
    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");

    /*
    {0,1}=100
    {0,2}=30
    {2,3}=10
    {3,4}=50
    {4,1}=9
    */

    vector<vector<int>> graph2 = {
        // clang-format off
                // 0      1        2        3        4
        /* 0 */{NO_EDGE, 100    , 30     , NO_EDGE, NO_EDGE}, 
        /* 1 */{100,    NO_EDGE, NO_EDGE, NO_EDGE, 9       }, 
        /* 2 */{30,     NO_EDGE, NO_EDGE, 10     , NO_EDGE }, 
        /* 3 */{NO_EDGE, NO_EDGE, 10     , NO_EDGE, 50     }, 
        /* 4 */{NO_EDGE, 9      , NO_EDGE, 50     , NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph2);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->2->3->4->1");

    // change the weight of the edge {1,4} to 10
    graph2[1][4] = 10;
    graph2[4][1] = 10;
    g.loadGraph(graph2);

    std::string res = Algorithms::shortestPath(g, 0, 1);
    CHECK((res == "0->1" || res == "0->2->3->4->1"));
}

TEST_CASE("Test shortestPath for UndirectedGraph weighted with negative weights") {
    /*
    if a graph contains a negative-weight edge, the shortest path between two vertices cannot be found
    */
    Graph g = Graph(false);

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 10,     -1},
        {10,     NO_EDGE, -5      },
        {-1,     -5,     NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);

    CHECK(Algorithms::shortestPath(g, 0, 1) == "Graph contains a negative-weight cycle");
}

TEST_CASE("Test isContainsCycle for DirectedGraph") {
    Graph g = Graph(true);

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 1,       NO_EDGE},
        {NO_EDGE, NO_EDGE, 1      },
        {1,       NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);
    CHECK(Algorithms::isContainsCycle(g) == true);

    vector<vector<int>> graph2 = {
        // clang-format off
        {NO_EDGE, 1,       NO_EDGE, NO_EDGE},
        {NO_EDGE, NO_EDGE, 1,       NO_EDGE},
        {NO_EDGE, NO_EDGE, NO_EDGE, 1      },
        {NO_EDGE, 1,       NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph2);
    CHECK(Algorithms::isContainsCycle(g) == true);

    vector<vector<int>> graph3 = {
        // clang-format off
        {NO_EDGE, 1,       NO_EDGE, NO_EDGE},
        {NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE},
        {1,       NO_EDGE, NO_EDGE, 1      },
        {NO_EDGE, NO_EDGE, 1,       NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph3);
    CHECK(Algorithms::isContainsCycle(g) == true);

    vector<vector<int>> graph4 = {
        // clang-format off
        {NO_EDGE, 1,       1      },
        {NO_EDGE, NO_EDGE, 1      },
        {NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph4);
    CHECK(Algorithms::isContainsCycle(g) == false);
}

TEST_CASE("Test isContainsCycle for UndirectedGraph") {
    Graph g = Graph(false);
    vector<vector<int>> graph = {
        {NO_EDGE, 1, NO_EDGE},
        {1, NO_EDGE, 1},
        {NO_EDGE, 1, NO_EDGE}};
    g.loadGraph(graph);
    CHECK(Algorithms::isContainsCycle(g) == false);
}

TEST_CASE("Test isBipartite for UndirectedGraph") {}

TEST_CASE("Test isBipartite for DirectedGraph") {}

TEST_CASE("Test negativeCycle for DirectedGraph") {}
