#include "Graph.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

namespace ariel {

void Graph::loadGraph(vector<vector<int>> ajdList) {
    this->ajdList.resize(ajdList.size());
    for (int i = 0; i < ajdList.size(); i++) {
        for (int j = 0; j < ajdList[i].size(); j++) {
            this->ajdList[i].push_back(ajdList[i][j]);

            if (i == j && ajdList[i][i] != 0) {
                throw invalid_argument("The diagonal of the matrix must be 0");
            }

            // check if the graph is weighted, directed or have negative edge weight
            if (ajdList[i][j] < 0) {
                haveNegativeEdgeWeight = true;
            }
            if (ajdList[i][j] != 1) {
                isWeighted = true;
            }
        }
    }
}

void Graph::printGraph() {
    int count_edges = 0;
    for (int i = 0; i < ajdList.size(); i++) {
        for (int j = 0; j < ajdList[i].size(); j++) {
            if (ajdList[i][j] != 0) {
                count_edges++;
            }
        }
    }

    // if the graph is undirected, the number of edges is divided by 2
    if (!isDirected) {
        count_edges /= 2;
    }

    cout << "Graph with " << ajdList.size() << " vertices and " << count_edges << " edges." << endl;
}

vector<vector<int>> Graph::getGraph() {
    return this->ajdList;
}

bool Graph::isWeightedGraph() {
    return this->isWeighted;
}
bool Graph::isDirectedGraph() {
    return this->isDirected;
}
bool Graph::isHaveNegativeEdgeWeight() {
    return this->haveNegativeEdgeWeight;
}
}  // namespace ariel