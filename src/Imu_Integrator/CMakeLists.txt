cmake_minimum_required(VERSION 2.8.3)
project(Imu_Integrator)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  message_generation
  std_msgs
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

generate_messages(
  DEPENDENCIES
    std_msgs
    visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Imu_Integrator_node src/Imu_Integrator.cpp)
target_link_libraries(Imu_Integrator_node ${catkin_LIBRARIES})

#using EIGEN
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

install(
  TARGETS
    Imu_Integrator_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
