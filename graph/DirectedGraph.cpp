#include "DirectedGraph.hpp"

#include <iostream>
#include <stdexcept>

using std::invalid_argument;

DirectedGraph::DirectedGraph() : Graph() {}

void DirectedGraph::loadGraph(vector<vector<int>> ajdList) {
    this->ajdList = ajdList;
    /*
     * update the isWeighted and haveNegativeEdgeWeight fields.
     * create an deep copy of the adjacency list.
     */

    // check if the graph is a square matrix
    // check if the diagonal of the matrix is 0
    for (int i = 0; i < ajdList.size(); i++) {
        if (ajdList.size() != ajdList[i].size()) {
            throw invalid_argument("Invalid graph: The graph is not a square matrix.");
        }
        if (ajdList[i][i] != NO_EDGE) {
            throw invalid_argument("The diagonal of the matrix must be NO_EDGE.");
        }

        this->ajdList[i][i] = NO_EDGE;  // the diagonal of the matrix must be NO_EDGE
    }

    // update the isWeighted and haveNegativeEdgeWeight fields if needed
    for (int i = 0; i < ajdList.size(); i++) {
        for (int j = 0; j < ajdList[i].size(); j++) {
            if (ajdList[i][j] != NO_EDGE) {
                if (ajdList[i][j] != 1) {
                    this->isWeighted = true;
                }
                if (ajdList[i][j] < 0) {
                    this->haveNegativeEdgeWeight = true;
                }
            }
        }
    }
}

void DirectedGraph::printGraph() {
    int count_edges = 0;
    for (int i = 0; i < ajdList.size(); i++) {
        for (int j = 0; j < ajdList[i].size(); j++) {
            if (ajdList[i][j] != NO_EDGE) {
                count_edges++;
            }
        }
    }

    std::cout << "Directed graph with " << ajdList.size() << " vertices and " << count_edges << " edges." << std::endl;
}