#include "MESPPProblem.h"

MESPPProblem::MESPPProblem(std::string df): data_folder(df)
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
    return n_vertices;
}

double MESPPProblem::getNRobots(){
    return n_robots;
}

double MESPPProblem::getNRounds(){
    return n_rounds;
}

int MESPPProblem::getStartVertexByRobot(int robot_id){
    return starting_vertices[robot_id];
}

Eigen::VectorXd MESPPProblem::getStartingBelief(){
    return starting_b;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MESPPProblem::getMMatrix(){
    return M_matrix;
}

std::set<int> MESPPProblem::getNeighbors(int vertex_id){
    return adjacencyList[vertex_id];
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MESPPProblem::getCMatrix(int robot_id, int vertex_id){
    return C_matrices[robot_id][vertex_id];
}

void MESPPProblem::loadStartInfo(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + data_folder + "/start.txt");
    std::string line;

    // read gamma
    std::getline(file, line);

    std::stringstream linestream(line);

    linestream >> gamma;

    //std::cout << gamma << std::endl;

    // read n_vertices
    std::getline(file, line);

    linestream = std::stringstream(line);

    linestream >> n_vertices;

    //std::cout << n_vertices << std::endl;

    // read starting_vertices
    std::getline(file, line);
    int robot_id = 1;
    int vertex;
    linestream = std::stringstream(line);

    while(linestream >> vertex){
        starting_vertices.insert({robot_id, vertex});
        robot_id += 1;
    }

    n_robots = robot_id - 1;

    // read starting_b
    std::getline(file, line);
    double val;
    linestream = std::stringstream(line);
    starting_b = Eigen::VectorXd(n_vertices + 1);
    int i = 0;
    while(linestream >> val){
        starting_b[i] = val;
        i += 1;
    }
    //std::cout << starting_b << std::endl;

    // read n_rounds
    std::getline(file, line);

    linestream = std::stringstream(line);

    linestream >> n_rounds;

    //std::cout << n_rounds << std::endl;
}

void MESPPProblem::loadMotionModel(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + data_folder + "/M.txt");
    std::string line;

    M_matrix.resize(n_vertices + 1, n_vertices + 1);

    std::getline(file, line);

    std::stringstream linestream(line);

    for(int i = 0; i < n_vertices + 1; i++){
        for(int j = 0; j < n_vertices + 1; j++){
            linestream >> M_matrix(i, j); 
        }
    }

    //std::cout << M_matrix << std::endl;
}

void MESPPProblem::loadAdjacencyList(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + data_folder + "/adj.txt");
    std::string line;

    while(std::getline(file, line)){

        if(line.length() == 0){
            break;
        }

        std::stringstream linestream(line);
        int vertex_id;
        linestream >> vertex_id;

        std::set<int> neighbors;
        neighbors.insert(vertex_id);
        int neighbor;

        while(linestream >> neighbor){
            neighbors.insert(neighbor);
        }

        adjacencyList.insert({vertex_id, neighbors});
    }

}

void MESPPProblem::loadCaptureMatrices(){
    std::ifstream file(ros::package::getPath("implicit_coordination") + "/data/" + data_folder + "/C.txt");
    std::string line;

    while(std::getline(file, line)){
        
        if(line.length() == 0){
            break;
        }

        std::stringstream linestream(line);
        int vertex_id, robot_id;
        linestream >> robot_id >> vertex_id;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C_matrix;
        C_matrix.resize(n_vertices + 1, n_vertices + 1);

        for(int i = 0; i < n_vertices + 1; i++){
            for(int j = 0; j < n_vertices + 1; j++){
                linestream >> C_matrix(i, j); 
            }
        }

        //std::cout << robot_id << vertex_id << std::endl;
        //std::cout << C_matrix << std::endl;

        C_matrices[robot_id][vertex_id] = C_matrix;

    }
}
