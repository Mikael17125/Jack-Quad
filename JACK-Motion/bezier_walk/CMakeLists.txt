cmake_minimum_required(VERSION 2.8.3)
project(bezier_walk)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  jack_framework_common
  jack_controller
  cmake_modules
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bezier_walk
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs jack_framework_common cmake_modules jack_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/bezierwalk.cpp
  src/IKSolver.cpp
  src/Trajectory.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
   ${Eigen3_LIBRARIES}
   ${Boost_LIBRARIES}
 )