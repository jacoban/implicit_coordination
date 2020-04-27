#ifndef INCLUDE_MESPPPROBLEM_H_
#define INCLUDE_MESPPPROBLEM_H_

#include <unordered_map>
#include <set>

#include <Eigen/Core>

class State
{

public:
    State(int step, double currObj, std::unordered_map<int, int> currVertices, Eigen::VectorXd belief);

    // an action is nothing but a legal move of a robot
    std::set<int> getNextActions(int robotId, const std::unordered_map<int, std::set<int>>& adjacencyList) const;

private:

    int step;

    double currObj;

    std::unordered_map<int, int> currVertices;

    Eigen::VectorXd belief;

};

#endif
