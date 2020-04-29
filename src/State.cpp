#include "State.h"

State::State(int step, double currObj, std::unordered_map<int, int> currVertices, Eigen::RowVectorXd belief, State* parent): 
step(step), currObj(currObj), currVertices(currVertices), belief(belief), parent(parent)
{
}

int State::getStep() const
{
    return step;
}

double State::getCurrObj() const
{
    return currObj;
}

Eigen::RowVectorXd State::getBelief() const
{
    return belief;
}

std::vector<int> State::getPathToThisState(int robotId)
{
    std::vector<int> path;

    State* currState = this;

    int count = 0;

    while(currState != nullptr)
    {
        path.push_back(currState->currVertices.at(robotId));
        currState = currState->parent;
    }

    std::reverse(path.begin(), path.end()); 

    return path;
}

int State::getCurrVertex(int robotId) const
{
    return currVertices.at(robotId);
}
