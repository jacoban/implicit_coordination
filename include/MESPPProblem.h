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

    int getStartVertexByRobot(int robot_id);

    Eigen::VectorXd getStartingBelief();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getMMatrix();

    std::set<int> getNeighbors(int vertex_id);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getCMatrix(int robot_id, int vertex_id);

private:

    std::string data_folder;

    double gamma;

    int n_vertices;

    int n_robots;

    int n_rounds;

    std::unordered_map<int, int> starting_vertices;

    Eigen::VectorXd starting_b;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M_matrix;

    std::unordered_map<int, std::set<int>> adjacencyList;

    std::unordered_map<int, std::unordered_map<int, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> C_matrices;     

    void loadStartInfo();

    void loadMotionModel();

    void loadAdjacencyList();

    void loadCaptureMatrices();

};

#endif
