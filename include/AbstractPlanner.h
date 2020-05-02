#ifndef INCLUDE_ABSTRACTPLANNER_H_
#define INCLUDE_ABSTRACTPLANNER_H_

#include <chrono>
#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "MESPPProblem.h"

class AbstractPlanner
{

public:
    AbstractPlanner(const MESPPProblem& problem);
    
    void makePlan();
    
    void logResults() const;

protected:

    // easy to implement centralized version if needed
    virtual void makePlanImpl() = 0;

    const MESPPProblem problem;

    double bestObj;

    std::vector<std::vector<int>> bestPaths;

private:

    std::chrono::duration<double> totTime;    

};

#endif
