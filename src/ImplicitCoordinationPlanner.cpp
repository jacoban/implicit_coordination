#include "ImplicitCoordinationPlanner.h"

ImplicitCoordinationPlanner::ImplicitCoordinationPlanner(const MESPPProblem& problem): AbstractPlanner(problem)
{
}


void ImplicitCoordinationPlanner::makePlanImpl()
{
    bestPaths = getInitialPaths();

    for(int i = 0; i < problem.getNRounds(); i++){
        std::cout << "Round of planning " << i << std::endl;
        planRound();
    }
}

void ImplicitCoordinationPlanner::planRound()
{
    for(int i = 0; i < problem.getNRobots(); i++){
        std::cout << "Updating path of robot " << i << std::endl;
        updateSingle(i);
    }
}

void ImplicitCoordinationPlanner::updateSingle(int robotId)
{
    exploredStates.push_back(problem.getInitialState());
    std::vector<State*> stack{exploredStates[0]};

    while(!stack.empty())
    {

        State* currStatePtr = stack.back();
        stack.pop_back();

        if(currStatePtr->getStep() == problem.getHorizon() && currStatePtr->getCurrObj() > bestObj)
        {
            //update
            std::cout << "Incumbent updated!" << std::endl; 
            bestObj = currStatePtr->getCurrObj();
            bestPaths[robotId] = currStatePtr->getPathToThisState(robotId);
        }
        else if(currStatePtr->getStep() < problem.getHorizon())
        {
            for(int nextVertex: problem.getNeighbors(currStatePtr->getCurrVertex(robotId)))
            {
                exploredStates.push_back(getNewState(currStatePtr, robotId, nextVertex));
                stack.push_back(exploredStates.back());
            }
        }

    }

    // free memory for next robot
    for(State* statePtr: exploredStates)
    {
        delete statePtr;
    }

    exploredStates.clear();
}

State* ImplicitCoordinationPlanner::getNewState(State* currStatePtr, int robotId, int newVertex) const
{
    std::vector<int> newVertices;

    int newStep = currStatePtr->getStep() + 1;
    
    for(int i = 0; i < problem.getNRobots(); i++)
    {
        if(i == robotId)
        {
            newVertices.push_back(newVertex);
        }
        else
        {
            newVertices.push_back(bestPaths[i][newStep]);
        }
    }

    // compute new belief vector
    
    // first motion model update
    Eigen::RowVectorXd newBelief = currStatePtr->getBelief() * problem.getMMatrix();

    // then apply capture matrices
    for(int i = 0; i < problem.getNRobots(); i++)
    {
        newBelief = newBelief * problem.getCMatrix(i, newVertices[i]);
    }

    double newObj = currStatePtr->getCurrObj() + pow(problem.getGamma(), newStep) * newBelief(0);

    return new State(newStep, newObj, newVertices, newBelief, currStatePtr);

}

std::vector<std::vector<int>> ImplicitCoordinationPlanner::getInitialPaths() const
{
    std::vector<int> initialVertices = problem.getStartingVertices();

    std::vector<std::vector<int>> initialPaths;

    for (int i = 0; i < initialVertices.size(); i++) 
    {
        int vertexId = initialVertices[i];

        std::vector<int> initialPath;

        for(int h=0; h <= problem.getHorizon(); h++)
        {
            initialPath.push_back(vertexId);
        }

        initialPaths.push_back(initialPath);
              
    }

    return initialPaths;
}
