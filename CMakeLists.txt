cmake_minimum_required(VERSION 3.10)
project(mloam)

#add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    rospy
    rosbag
    std_msgs
    image_transport
    cv_bridge
    tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS include
    LIBRARIES ndt_omp
)

add_library(
    ndt_omp
    include/pclomp/src/voxel_grid_covariance_omp.cpp
    include/pclomp/src/ndt_omp.cpp
    include/pclomp/src/gicp_omp.cpp
)

add_executable(preprocessing src/preprocessing.cpp)
target_link_libraries(preprocessing ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ndt_omp)
