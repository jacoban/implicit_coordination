#include "MESPPProblem.h"

MESPPProblem::MESPPProblem(std::string df): dataFolder(df)
{
    loadStartInfo();
    loadMotionModel();
    loadCaptureMatrices();
    loadAdjacencyList();
}

double MESPPProblem::getGamma(){
    return gamma;
}

double MESPPProblem::getNVertices(){
    return nVertices;
}

double MESPPProblem::getNRobots(){
    return nRobots;
}

double MESPPProblem::getNRounds(){
    return nRounds;
}

int MESPPProblem::getStartVertexByRobot(int robotId){
    return startingVertices[robotId];
}

Eigen::VectorXd MESPPProblem::getStartingBelief(){
    return startingBelief;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MESPPProblem::getMMatrix(){
    return MMatrix;
}

std::set<int> MESPPProblem::getNeighbors(int vertexId){
    return adjacencyList[vertexId];
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MESPPProblem::getCMatrix(int robotId, int vertexId){
    return CMatrices[robotId][vertexId];
}

void MESPPProblem::loadStartInfo(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + dataFolder + "/start.txt");
    std::string line;

    // read gamma
    std::getline(file, line);

    std::stringstream linestream(line);

    linestream >> gamma;

    //std::cout << gamma << std::endl;

    // read nVertices
    std::getline(file, line);

    linestream = std::stringstream(line);

    linestream >> nVertices;

    //std::cout << nVertices << std::endl;

    // read startingVertices
    std::getline(file, line);
    int robotId = 1;
    int vertex;
    linestream = std::stringstream(line);

    while(linestream >> vertex){
        startingVertices.insert({robotId, vertex});
        robotId += 1;
    }

    nRobots = robotId - 1;

    // read startingBelief
    std::getline(file, line);
    double val;
    linestream = std::stringstream(line);
    startingBelief = Eigen::VectorXd(nVertices + 1);
    int i = 0;
    while(linestream >> val){
        startingBelief[i] = val;
        i += 1;
    }
    //std::cout << startingBelief << std::endl;

    // read nRounds
    std::getline(file, line);

    linestream = std::stringstream(line);

    linestream >> nRounds;

    //std::cout << nRounds << std::endl;
}

void MESPPProblem::loadMotionModel(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + dataFolder + "/M.txt");
    std::string line;

    MMatrix.resize(nVertices + 1, nVertices + 1);

    std::getline(file, line);

    std::stringstream linestream(line);

    for(int i = 0; i < nVertices + 1; i++){
        for(int j = 0; j < nVertices + 1; j++){
            linestream >> MMatrix(i, j); 
        }
    }

    //std::cout << MMatrix << std::endl;
}

void MESPPProblem::loadAdjacencyList(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + dataFolder + "/adj.txt");
    std::string line;

    while(std::getline(file, line)){

        if(line.length() == 0){
            break;
        }

        std::stringstream linestream(line);
        int vertexId;
        linestream >> vertexId;

        std::set<int> neighbors;
        neighbors.insert(vertexId);
        int neighbor;

        while(linestream >> neighbor){
            neighbors.insert(neighbor);
        }

        adjacencyList.insert({vertexId, neighbors});
    }

}

void MESPPProblem::loadCaptureMatrices(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + dataFolder + "/C.txt");
    std::string line;

    while(std::getline(file, line)){
        
        if(line.length() == 0){
            break;
        }

        std::stringstream linestream(line);
        int vertexId, robotId;
        linestream >> robotId >> vertexId;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> CMatrix;
        CMatrix.resize(nVertices + 1, nVertices + 1);

        for(int i = 0; i < nVertices + 1; i++){
            for(int j = 0; j < nVertices + 1; j++){
                linestream >> CMatrix(i, j); 
            }
        }

        //std::cout << robotId << vertexId << std::endl;
        //std::cout << CMatrix << std::endl;

        CMatrices[robotId][vertexId] = CMatrix;

    }
}
