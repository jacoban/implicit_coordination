cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp)
project(implicit_coordination)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  roslib
)

# ----------------------- INCLUDES ----------------------------

include_directories(include ${catkin_INCLUDE_DIRS})

# ----------------------- LIBRARIES ---------------------------

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ------------------------ MESSAGES ----------------------------

  
# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE ic_SOURCE src/*.cpp)
list(REMOVE_ITEM ic_SOURCE "src/implicit_coordination_main.cpp")

add_library(implicit_coordination_planner ${ic_SOURCE})
                        
target_link_libraries(implicit_coordination_planner ${catkin_LIBRARIES})     

add_executable(implicit_coordination "src/implicit_coordination_main.cpp") 
target_link_libraries(implicit_coordination implicit_coordination_planner ${catkin_LIBRARIES})       
