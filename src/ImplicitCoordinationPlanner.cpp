#include "ImplicitCoordinationPlanner.h"

ImplicitCoordinationPlanner::ImplicitCoordinationPlanner(const MESPPProblem& problem): AbstractPlanner(problem)
{
}


void ImplicitCoordinationPlanner::makePlanImpl()
{
    std::unordered_map<int, std::vector<int>> currPaths = getInitialPaths();

    for(int i = 0; i < problem.getNRounds(); i++){
        planRound(currPaths);
    }
}

void ImplicitCoordinationPlanner::planRound(std::unordered_map<int, std::vector<int>>& currPaths)
{
    for(int i = 1; i <= problem.getNRobots(); i++){
        updateSingle(i, currPaths);
    }
}

void ImplicitCoordinationPlanner::updateSingle(int robotId, std::unordered_map<int, std::vector<int>>& currPaths)
{
    State currState = problem.getInitialState();
    std::vector<State> stack{currState};

    while(!stack.empty())
    {

    }
}

std::unordered_map<int, std::vector<int>> ImplicitCoordinationPlanner::getInitialPaths()
{
    std::unordered_map<int, int> initialVertices = problem.getStartingVertices();

    std::unordered_map<int, std::vector<int>> initialPaths;

    for (const auto &pair : initialVertices) 
    {
        int robotId = pair.first;
        int vertexId = pair.second;

        std::vector<int> initialPath;

        for(int h=0; h <= problem.getHorizon(); h++)
        {
            initialPath.push_back(vertexId);
        }

        initialPaths.insert({robotId, initialPath});
              
    }

    return initialPaths;
}
