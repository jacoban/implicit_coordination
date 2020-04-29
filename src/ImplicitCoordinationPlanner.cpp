#include "ImplicitCoordinationPlanner.h"

ImplicitCoordinationPlanner::ImplicitCoordinationPlanner(const MESPPProblem& problem): AbstractPlanner(problem)
{
}


void ImplicitCoordinationPlanner::makePlanImpl()
{
    bestPaths = getInitialPaths();

    for(int i = 0; i < problem.getNRounds(); i++){
        planRound();
    }
}

void ImplicitCoordinationPlanner::planRound()
{
    for(int i = 1; i <= problem.getNRobots(); i++){
        updateSingle(i);
    }
}

void ImplicitCoordinationPlanner::updateSingle(int robotId)
{
    State initState = problem.getInitialState();
    std::vector<State> stack{initState};

    while(!stack.empty())
    {
        State currState = stack.back();
        stack.pop_back();

        if(currState.getStep() == problem.getHorizon() && currState.getCurrObj() > bestObj)
        {
            //update 
            bestObj = currState.getCurrObj();
            bestPaths[robotId] = currState.getPathToThisState(robotId);
        }
        else if(currState.getStep() < problem.getHorizon())
        {
            for(int nextVertex: problem.getNeighbors(currState.getCurrVertex(robotId)))
            {
                //stack.push_back(getNewState(currState, robotId, nextVertex));
            }
        }

    }
}

State ImplicitCoordinationPlanner::getNewState(State& currState, int robotId, int newVertex)
{
    std::unordered_map<int, int> newVertices;

    int newStep = currState.getStep() + 1;
    
    for(int i = 1; i <= problem.getNRobots(); i++)
    {
        if(i == robotId)
        {
            newVertices.insert({robotId, newVertex});
        }
        else
        {
            newVertices.insert({i, bestPaths.at(i)[newStep]});
        }
    }

    // compute new belief vector
    
    // first motion model update
    Eigen::VectorXd newBelief = currState.getBelief() * problem.getMMatrix();

    // then apply capture matrices
    for(int i = 1; i <= problem.getNRobots(); i++)
    {
        newBelief *= problem.getCMatrix(i, newVertices.at(i));
    }

    double newObj = currState.getCurrObj() + pow(problem.getGamma(), newStep) * newBelief(0);

    return State(newStep, newObj, newVertices, newBelief, &currState);

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
