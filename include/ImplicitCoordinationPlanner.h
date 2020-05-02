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

    std::vector<std::vector<int>> getInitialPaths() const;

    std::vector<State*> exploredStates;

    void planRound();

    void updateSingle(int robotId);
    
    State* getNewState(State* currState, int robotId, int nextVertex) const;

};

#endif
