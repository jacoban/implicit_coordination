#include "AbstractPlanner.h"

AbstractPlanner::AbstractPlanner(const MESPPProblem& problem): problem(problem)
{
}


void AbstractPlanner::makePlan()
{

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    makePlanImpl();

    totTime = std::chrono::steady_clock::now() - t0;

}

void AbstractPlanner::logResults() const
{
    std::cout << "Saving results..." << std::endl;

    std::string dataFolder = problem.getDataFolder();
    //use same name for log folder
    std::string logFolderPath = ros::package::getPath("implicit_coordination") + "/log/" + dataFolder;

    struct stat st = {0};

    if (stat(logFolderPath.c_str(), &st) == -1) 
    {
        std::cout << "Creating log folder as it does not exist" << std::endl;
        mkdir(logFolderPath.c_str(), 0700);
    }

    std::ofstream file(logFolderPath + "/solver_data.txt");

    file << totTime.count();
    file << "\n";
    file << bestObj;

    file.close();

    file.open(logFolderPath + "/paths.txt");
    
    for (const auto &path : bestPaths) 
    {
        file << path.first << " ";
        for(int vertex: path.second){
            file << vertex << " ";
        }
        file << "\n";        
    }
    
    file.close();    
}
