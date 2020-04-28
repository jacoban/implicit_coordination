#ifndef INCLUDE_ABSTRACTPLANNER_H_
#define INCLUDE_ABSTRACTPLANNER_H_

#include <chrono>
#include <unordered_map>
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
    //virtual bool makePlanImpl() = 0;

    MESPPProblem problem;

    double bestObj;

    std::unordered_map<int, std::vector<int>> bestPaths;

private:

    std::chrono::duration<double> totTime;    

};

#endif
