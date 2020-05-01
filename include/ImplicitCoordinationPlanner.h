#ifndef INCLUDE_IMPLICITCOORDINATIONPLANNER_H_
#define INCLUDE_IMPLICITCOORDINATIONPLANNER_H_

#include <math.h> 

#include "AbstractPlanner.h"

class ImplicitCoordinationPlanner : public AbstractPlanner
{

public:

    ImplicitCoordinationPlanner(const MESPPProblem& problem);


protected:

    virtual void makePlanImpl() override;

private:

    std::unordered_map<int, std::vector<int>> getInitialPaths();

    std::vector<State*> exploredStates;

    void planRound();

    void updateSingle(int robotId);
    
    State* getNewState(State* currState, int robotId, int nextVertex);

};

#endif
