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

#include "State.h"

class MESPPProblem
{

public:

    MESPPProblem(std::string data_folder);

    std::string getDataFolder() const;

    int getHorizon() const;

    double getGamma() const;

    double getNVertices() const;

    double getNRobots() const;

    double getNRounds() const;

    int getStartVertexByRobot(int robotId) const;

    std::vector<int> getStartingVertices() const;

    Eigen::RowVectorXd getStartingBelief() const;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getMMatrix() const;

    std::set<int> getNeighbors(int vertexId) const;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getCMatrix(int robotId, int vertexId) const;

    State* getInitialState() const;

private:

    std::string dataFolder;

    int horizon;

    double gamma;

    int nVertices;

    int nRobots;

    int nRounds;

    std::vector<int> startingVertices;

    Eigen::RowVectorXd startingBelief;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MMatrix;

    std::unordered_map<int, std::set<int>> adjacencyList;

    std::unordered_map<int, std::unordered_map<int, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> CMatrices;     

    void loadStartInfo();

    void loadMotionModel();

    void loadAdjacencyList();

    void loadCaptureMatrices();

};

#endif
