cmake_minimum_required(VERSION 3.10)
project(mloam)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

set(CMAKE_CXX_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    geodesy
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
find_package(GTSAM REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library(
    ndt_omp
    include/pclomp/src/voxel_grid_covariance_omp.cpp
    include/pclomp/src/ndt_omp.cpp
    include/pclomp/src/gicp_omp.cpp
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
    DEPENDS EIGEN3 PCL Ceres
    INCLUDE_DIRS include
    LIBRARIES ndt_omp
)

add_executable(laserProcessing src/laserProcessing.cpp)
target_link_libraries(laserProcessing ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(laserOdometry src/laserOdometry.cpp src/keyframe.cpp src/map_generator.cpp src/loop_detector.cpp src/lidar.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam ndt_omp)

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_subdirectory(test)