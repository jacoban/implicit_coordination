#include "State.h"

State::State(int step, double currObj, std::unordered_map<int, int> currVertices, Eigen::VectorXd belief, State* parent): 
step(step), currObj(currObj), currVertices(currVertices), belief(belief), parent(parent)
{
}

std::set<int> State::getNextActions(int robotId, const std::unordered_map<int, std::set<int>>& adjacencyList) const
{
    return adjacencyList.at(currVertices.at(robotId));
}

std::vector<int> State::getPathToThisState(int robotId)
{
    std::vector<int> path;

    State* currState = this;

    while(currState != nullptr)
    {
        path.push_back(currState->currVertices.at(robotId));
        currState = currState->parent;
    }

    std::reverse(path.begin(), path.end()); 

    return path;
}
