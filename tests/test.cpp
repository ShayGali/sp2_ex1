#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "../algorithms/Algorithms.hpp"
#include "../graph/Graph.hpp"
#include "doctest.h"

using namespace shayg;
TEST_CASE("Test loadGraph for Directed Graph") {
    Graph g(true);

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
    CHECK_THROWS_AS(g.loadGraph(graph2), std::invalid_argument);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the diagonal of the matrix is not NO_EDGE
    vector<vector<int>> graph3 = {
        // clang-format off
            {NO_EDGE, 1,       1      },
            {1,       1,       1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph3), std::invalid_argument);

    // empty graph
    vector<vector<int>> graph4 = {};
    g.loadGraph(graph4);
    CHECK(std::equal(graph4.begin(), graph4.end(), g.getGraph().begin()));
}

TEST_CASE("Test loadGraph for undirected graph") {
    Graph g(false);
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
    CHECK_THROWS_AS(g.loadGraph(graph2), std::invalid_argument);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // the diagonal of the matrix is not NO_EDGE
    vector<vector<int>> graph3 = {
        // clang-format off
            {NO_EDGE, 1,       1      },
            {1,       1,       1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph3), std::invalid_argument);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // non-symmetric matrix will cause an exception
    vector<vector<int>> graph4 = {
        // clang-format off
            {NO_EDGE, 2,       1      },
            {1,       NO_EDGE, 1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    CHECK_THROWS_AS(g.loadGraph(graph4), std::invalid_argument);
}

TEST_CASE("Test printGraph") {
    std::stringstream buffer;
    std::streambuf* prevcoutbuf = std::cout.rdbuf(buffer.rdbuf());

    Graph g_dir(true);
    Graph g_undir(false);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 1,       1      },
        {1,       NO_EDGE, 1      },
        {1,       1,       NO_EDGE}
        // clang-format on
    };

    g_dir.loadGraph(graph);

    g_dir.printGraph();

    CHECK(buffer.str() == "Directed graph with 3 vertices and 6 edges.\n");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    buffer.str("");
    g_undir.loadGraph(graph);
    g_undir.printGraph();

    CHECK(buffer.str() == "Undirected graph with 3 vertices and 3 edges.\n");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // empty graph
    buffer.str("");

    vector<vector<int>> emptyGraph = {};
    g_dir.loadGraph(emptyGraph);
    g_dir.printGraph();
    CHECK(buffer.str() == "Directed graph with 0 vertices and 0 edges.\n");

    buffer.str("");
    g_undir.loadGraph(emptyGraph);
    g_undir.printGraph();
    CHECK(buffer.str() == "Undirected graph with 0 vertices and 0 edges.\n");

    // Restore std::cout to its original buffer
    std::cout.rdbuf(prevcoutbuf);
}

TEST_CASE("Test isConnected for directed graph") {
    Graph g(true);

    // empty graph is connected
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isConnected(g) == true);

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

TEST_CASE("Test isConnected for undirected graph") {
    Graph g(false);
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

    // empty graph is connected
    vector<vector<int>> graph4 = {};
    g.loadGraph(graph4);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Error check shortestPath") {
    Graph g(true);
    vector<vector<int>> graph = {
        // clang-format off
            {NO_EDGE, 1,       1      },
            {1,       NO_EDGE, 1      },
            {1,       1,       NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);

    // destination vertex is not in the graph
    CHECK_THROWS_AS(Algorithms::shortestPath(g, 0, 3), std::invalid_argument);

    // source vertex is not in the graph
    CHECK_THROWS_AS(Algorithms::shortestPath(g, 3, 1), std::invalid_argument);

    // source and destination vertices are not in the graph
    CHECK_THROWS_AS(Algorithms::shortestPath(g, 12, 3), std::invalid_argument);
}

TEST_CASE("Test shortestPath for directed graph unweighted") {
    Graph g(true);
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

TEST_CASE("Test shortestPath for directed graph weighted non-negative") {
    Graph g(true);

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

TEST_CASE("Test shortestPath for directed graph weighted with negative weights") {
    Graph g(true);

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

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vector<vector<int>> graph4 = {
        // clang-format off
    { NO_EDGE, 5      , 3      , NO_EDGE, NO_EDGE }, // A
    { NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE }, // B
    { NO_EDGE, NO_EDGE, NO_EDGE, -2     , NO_EDGE }, // C
    { NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, -1      }, // D
    { NO_EDGE, NO_EDGE, -1     , NO_EDGE, NO_EDGE }  // E
        // clang-format on
    };
    g.loadGraph(graph4);
    CHECK(Algorithms::shortestPath(g, 0, 1) == "Graph contains a negative-weight cycle");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

TEST_CASE("Test shortestPath for undirected graph unweighted") {
    Graph g(false);
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

TEST_CASE("Test shortestPath for undirected graph weighted non-negative") {
    Graph g(false);

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

TEST_CASE("Test shortestPath for undirected graph weighted with negative weights") {
    /*
    A -(-10)- B
    this graph dont have a negative cycle
    */
    Graph g(false);

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, -10}, 
        {-10,    NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);

    CHECK(Algorithms::shortestPath(g, 0, 1) == "0->1");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // graph with a negative cycle
    vector<vector<int>> graph1 = {
        // clang-format off
        {NO_EDGE, -1,      1     },
        {-1,     NO_EDGE, -1     },
        {1,      -1,      NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph1);
    CHECK(Algorithms::shortestPath(g, 0, 2) == "Graph contains a negative-weight cycle");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   vector<vector<int>> graph2 = {
        // clang-format off
    //      0        1        2        3      4        5     6      7        8        9       10       11
    /* 0*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 1*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 2*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 4*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 5*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 6*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 7*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 8*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /* 9*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /*10*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /*12*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE , NO_EDGE},
    /*13*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, 10     , -1     , 1      , NO_EDGE},
    /*14*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,10     , NO_EDGE, 10     , NO_EDGE, NO_EDGE},
    /*15*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,-1     , 10     , NO_EDGE, NO_EDGE, 2      },
    /*16*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,1      , NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE},
    /*17*/{NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE,NO_EDGE, NO_EDGE, 2      , NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph2);
    CHECK(Algorithms::shortestPath(g, 12, 13) == "12->11->13");
}
TEST_CASE("Test isContainsCycle for directed graph") {
    Graph g(true);

    // empty graph
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isContainsCycle(g) == "-1");
    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 1,       NO_EDGE},
        {NO_EDGE, NO_EDGE, 1      },
        {1,       NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph);
    CHECK(Algorithms::isContainsCycle(g) == "0->1->2->0");

    vector<vector<int>> graph2 = {
        // clang-format off
        {NO_EDGE, 1,       NO_EDGE, NO_EDGE},
        {NO_EDGE, NO_EDGE, 1,       NO_EDGE},
        {NO_EDGE, NO_EDGE, NO_EDGE, 1      },
        {NO_EDGE, 1,       NO_EDGE, NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph2);
    CHECK(Algorithms::isContainsCycle(g) == "1->2->3->1");

    vector<vector<int>> graph3 = {
        // clang-format off
        {NO_EDGE, 1,       NO_EDGE, NO_EDGE},
        {NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE},
        {1,       NO_EDGE, NO_EDGE, 1      },
        {NO_EDGE, NO_EDGE, 1,       NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph3);
    CHECK(Algorithms::isContainsCycle(g) == "2->3->2");

    vector<vector<int>> graph4 = {
        // clang-format off
        {NO_EDGE, 1,       1      },
        {NO_EDGE, NO_EDGE, 1      },
        {NO_EDGE, NO_EDGE, NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph4);
    CHECK(Algorithms::isContainsCycle(g) == "-1");

    vector<vector<int>> graph5 = {
        // clang-format off
        {NO_EDGE, 1      , 2         ,NO_EDGE, NO_EDGE},
        {1      , NO_EDGE, 3         ,NO_EDGE, NO_EDGE},
        {2      , 3      , NO_EDGE   ,4      , NO_EDGE},
        {NO_EDGE, NO_EDGE, 4         ,NO_EDGE, 5      },
        {NO_EDGE, NO_EDGE, NO_EDGE   ,5      , NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph5);
    CHECK(Algorithms::isContainsCycle(g) == "0->1->0");

    vector<vector<int>> graph6 = {
        // clang-format off
        {NO_EDGE, NO_EDGE, NO_EDGE, 1      },
        {NO_EDGE, NO_EDGE, 1      , NO_EDGE},
        {1      , NO_EDGE, NO_EDGE, NO_EDGE},
        {NO_EDGE, 1      , NO_EDGE, NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph6);
    CHECK(Algorithms::isContainsCycle(g) == "0->3->1->2->0");
}

TEST_CASE("Test isContainsCycle for undirected graph") {
    Graph g(false);

    // empty graph
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isContainsCycle(g) == "-1");

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 1      , NO_EDGE},
        {1      , NO_EDGE, 1      },
        {NO_EDGE, 1      , NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    CHECK(Algorithms::isContainsCycle(g) == "-1");

    vector<vector<int>> graph2 = {
        // clang-format off
        {NO_EDGE, 1      , 1      },
        {1      , NO_EDGE, 1      },
        {1      , 1      , NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph2);
    CHECK(Algorithms::isContainsCycle(g) == "0->1->2->0");

    vector<vector<int>> graph3 = {
        // clang-format off
    {NO_EDGE, 1      , NO_EDGE, NO_EDGE},
    {1      , NO_EDGE, 1      , 1      },
    {NO_EDGE, 1      , NO_EDGE, 1      },
    {NO_EDGE, 1      , 1      , NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph3);
    CHECK(Algorithms::isContainsCycle(g) == "1->2->3->1");
}

TEST_CASE("Test isBipartite for undirected graph") {
    Graph g(false);

    // empty graph
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={}, B={}");

    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE, 1      , 1      ,1      },
        {1      , NO_EDGE, 1      ,NO_EDGE},
        {1      , 1      , NO_EDGE,1      },
        {1      , NO_EDGE, 1      ,NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);

    CHECK(Algorithms::isBipartite(g) == "The graph is not bipartite");

    vector<vector<int>> graph2 = {
        // clang-format off
        {NO_EDGE, 1      , NO_EDGE,1      },
        {1      , NO_EDGE, NO_EDGE,NO_EDGE},
        {NO_EDGE, NO_EDGE, NO_EDGE,1      },
        {1      , NO_EDGE, 1      ,NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph2);

    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2}, B={1,3}");
}

TEST_CASE("Test isBipartite for directed graph") {
    Graph g(true);

    // empty graph
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={}, B={}");

    // 0 <-- 1
    vector<vector<int>> graph = {
        // clang-format off
        {NO_EDGE,NO_EDGE},
        {1      ,NO_EDGE}
        // clang-format on
    };
    g.loadGraph(graph);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0}, B={1}");

    // // 0 <-- 1 <-- 2
    vector<vector<int>> graph2 = {
        // clang-format off
        {NO_EDGE,NO_EDGE,NO_EDGE},
        {1      ,NO_EDGE,NO_EDGE},
        {NO_EDGE,1      ,NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph2);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2}, B={1}");

    // 0 <-- 1 --> 2
    vector<vector<int>> graph3 = {
        // clang-format off
        {NO_EDGE,NO_EDGE,NO_EDGE},
        {1      ,NO_EDGE,1     },
        {NO_EDGE,NO_EDGE,NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph3);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2}, B={1}");

    // triangle
    vector<vector<int>> graph4 = {
        // clang-format off
        {NO_EDGE,1      ,NO_EDGE},
        {NO_EDGE,NO_EDGE,1     },
        {1      ,NO_EDGE,NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph4);
    CHECK(Algorithms::isBipartite(g) == "The graph is not bipartite");
}

TEST_CASE("Test negativeCycle for directed graph") {
    Graph g(true);

    // empty graph
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle");

    vector<vector<int>> graph1 = {
        // clang-format off
        {NO_EDGE, -1      ,NO_EDGE},
        {NO_EDGE, NO_EDGE,-1      },
        {1      , NO_EDGE,NO_EDGE }
        // clang-format on
    };

    g.loadGraph(graph1);
    CHECK(Algorithms::negativeCycle(g) == "0->1->2->0");

    vector<vector<int>> graph2 = {
        // clang-format off
    { NO_EDGE, 5      , 3      , NO_EDGE, NO_EDGE }, // A
    { NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE }, // B
    { NO_EDGE, NO_EDGE, NO_EDGE, -2     , NO_EDGE }, // C
    { NO_EDGE, NO_EDGE, NO_EDGE, NO_EDGE, -1      }, // D
    { NO_EDGE, NO_EDGE, -1     , NO_EDGE, NO_EDGE }  // E
        // clang-format on
    };

    g.loadGraph(graph2);
    CHECK(Algorithms::negativeCycle(g) == "3->4->2->3");
}

TEST_CASE("Test negativeCycle for undirected graph") {
    Graph g(false);

    // empty graph
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle");

    vector<vector<int>> graph1 = {
        // clang-format off
        {NO_EDGE, -1     },
        {-1     , NO_EDGE}
        // clang-format on
    };

    g.loadGraph(graph1);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle");
}
