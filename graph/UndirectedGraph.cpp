#include "UndirectedGraph.hpp"

#include <iostream>
#include <stdexcept>

using std::invalid_argument;

UndirectedGraph::UndirectedGraph() : Graph() {}

void UndirectedGraph::loadGraph(vector<vector<int>> ajdList) {
    this->ajdList = ajdList;
    /*
     * update the isWeighted and haveNegativeEdgeWeight fields.
     * create an deep copy of the adjacency list.
     */

    // check if the graph is a square matrix
    // check if the diagonal of the matrix is 0
    for (int i = 0; i < ajdList.size(); i++) {
        if (ajdList.size() != ajdList[i].size()) {
            throw invalid_argument("Invalid graph: The graph is not a square matrix.(" + std::to_string(i) + "th row has " + std::to_string(ajdList[i].size()) + " elements.)");
        }
        if (ajdList[i][i] != NO_EDGE) {
            throw invalid_argument("The diagonal of the matrix must be NO_EDGE. (the " + std::to_string(i) + "th node is not a NO_EDGE)");
        }

        this->ajdList[i][i] = NO_EDGE;  // the diagonal of the matrix must be NO_EDGE
    }

    // update the isWeighted and haveNegativeEdgeWeight fields if needed
    for (int i = 0; i < ajdList.size(); i++) {
        for (int j = i + 1; j < ajdList[i].size(); j++) {
            if (ajdList[i][j] != NO_EDGE) {
                if (ajdList[i][j] != 1) {
                    this->isWeighted = true;
                }
                if (ajdList[i][j] < 0) {
                    this->haveNegativeEdgeWeight = true;
                }
                if (ajdList[i][j] != ajdList[j][i]) {
                    throw invalid_argument("Invalid graph: The graph is not symmetric.(mat[" + std::to_string(i) + "][" + std::to_string(j) + "] = " + std::to_string(ajdList[i][j]) + " and mat[" + std::to_string(j) + "][" + std::to_string(i) + "] = " + std::to_string(ajdList[j][i]) + ")");
                }
            }
        }
    }
}

void UndirectedGraph::printGraph() {
    int count_edges = 0;
    for (int i = 0; i < ajdList.size(); i++) {
        // only count the upper triangle of the matrix
        for (int j = i + 1; j < ajdList[i].size(); j++) {
            if (ajdList[i][j] != NO_EDGE) {
                count_edges++;
            }
        }

        std::cout << "Undirected graph with " << ajdList.size() << " vertices and " << count_edges << " edges." << std::endl;
    }
}