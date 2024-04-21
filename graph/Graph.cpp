#include "Graph.hpp"

Graph::Graph() {
    this->isWeighted = false;
    this->haveNegativeEdgeWeight = false;
}

vector<vector<int>> Graph::getGraph() {
    return this->ajdList;
}

bool Graph::isWeightedGraph() {
    return this->isWeighted;
}

bool Graph::isHaveNegativeEdgeWeight() {
    return this->haveNegativeEdgeWeight;
}