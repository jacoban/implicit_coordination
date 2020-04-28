#include <stdlib.h>
#include <iostream>

#include "MESPPProblem.h"
#include "ImplicitCoordinationPlanner.h"

int main(int argc, char** argv)
{

    if(argc < 2){
        std::cout << "The program needs a folder name to load / save data. See the docs. Exiting." << std::endl;
        exit(1);
    }

    std::string data_folder = argv[1];

    MESPPProblem problem(data_folder);

    ImplicitCoordinationPlanner planner(problem);

    planner.makePlan();

    planner.logResults();

    exit(0);
}
