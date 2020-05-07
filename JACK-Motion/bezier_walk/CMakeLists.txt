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
  eigen_conversions
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bezier_walk
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs jack_framework_common cmake_modules jack_controller
  DEPENDS Boost EIGEN3
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

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)