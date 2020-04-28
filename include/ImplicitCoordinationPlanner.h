#ifndef INCLUDE_IMPLICITCOORDINATIONLANNER_H_
#define INCLUDE_IMPLICITCOORDINATIONPLANNER_H_

#include "AbstractPlanner.h"

class ImplicitCoordinationPlanner : public AbstractPlanner
{

public:

    ImplicitCoordinationPlanner(const MESPPProblem& problem);


protected:

    virtual void makePlanImpl() override;

private:

    std::unordered_map<int, std::vector<int>> getInitialPaths();

    void planRound(std::unordered_map<int, std::vector<int>>& currPaths);

    void updateSingle(int robotId, std::unordered_map<int, std::vector<int>>& currPaths);

};

#endif
