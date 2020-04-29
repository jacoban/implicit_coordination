#ifndef INCLUDE_STATE_H_
#define INCLUDE_STATE_H_

#include <unordered_map>
#include <vector>
#include <set>

#include <Eigen/Core>

class State
{

public:
    State(int step, double currObj, std::unordered_map<int, int> currVertices, Eigen::VectorXd belief, State* parent);

    int getStep() const;

    double getCurrObj() const;

    Eigen::VectorXd getBelief() const;

    std::vector<int> getPathToThisState(int robotId);

    int getCurrVertex(int robotId) const;

    // could also add the centralized variant that retrieves the joint paths
    // not needed by ImplicitCoordinationPlanner

private:

    int step;

    double currObj;

    std::unordered_map<int, int> currVertices;

    Eigen::VectorXd belief;

    State* parent;

};

#endif
