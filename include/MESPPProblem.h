#ifndef INCLUDE_MESPPPROBLEM_H_
#define INCLUDE_MESPPPROBLEM_H_

#include <string>
#include <iostream>
#include <fstream>
#include <set>
#include <utility>
#include <unordered_map>

#include <Eigen/Core>
#include <ros/package.h>

class MESPPProblem
{

public:

    MESPPProblem(std::string data_folder);

    double getGamma();

    double getNVertices();

    double getNRobots();

    double getNRounds();

    int getStartVertexByRobot(int robotId);

    Eigen::VectorXd getStartingBelief();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getMMatrix();

    std::set<int> getNeighbors(int vertexId);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getCMatrix(int robotId, int vertexId);

private:

    std::string dataFolder;

    double gamma;

    int nVertices;

    int nRobots;

    int nRounds;

    std::unordered_map<int, int> startingVertices;

    Eigen::VectorXd startingBelief;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MMatrix;

    std::unordered_map<int, std::set<int>> adjacencyList;

    std::unordered_map<int, std::unordered_map<int, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> CMatrices;     

    void loadStartInfo();

    void loadMotionModel();

    void loadAdjacencyList();

    void loadCaptureMatrices();

};

#endif
