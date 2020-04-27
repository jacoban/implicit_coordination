#include "State.h"

State::State(int step, double currObj, std::unordered_map<int, int> currVertices, Eigen::VectorXd belief): 
step(step), currObj(currObj), currVertices(currVertices), belief(belief)
{
}

std::set<int> State::getNextActions(int robotId, const std::unordered_map<int, std::set<int>>& adjacencyList){
    return adjacencyList[currVertices[robotId]];
}
