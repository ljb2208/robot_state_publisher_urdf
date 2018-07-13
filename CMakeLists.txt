cmake_minimum_required(VERSION 2.8.3)
project(robot_state_publisher_urdf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS roscpp rosconsole rostime tf tf2_ros tf2_kdl kdl_parser
)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_solver
  INCLUDE_DIRS include
  DEPENDS roscpp rosconsole rostime tf2_ros tf2_kdl kdl_parser orocos_kdl urdfdom_headers
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_solver
  src/robot_state_publisher_urdf.cpp src/treefksolverposfull_recursive.cpp
)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/robot_state_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_solver robot_state_publisher_urdf ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})