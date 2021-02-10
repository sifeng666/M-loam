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
    fast_gicp
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
    DEPENDS EIGEN3 PCL Ceres
    INCLUDE_DIRS include
    LIBRARIES
)

add_executable(laserProcessing src/laserProcessing.cpp)
target_link_libraries(laserProcessing ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(laserOdometry
        src/laserOdometry.cpp
        src/lidar.cpp

        include/tools/keyframe.hpp
        include/tools/map_generator.hpp
        include/tools/loop_detector.hpp
        include/tools/calibration.hpp
        include/tools/registration.hpp
        include/tools/calibration_graph.cpp
        include/tools/camodocal/src/PlanarHandEyeCalibration.cc
        include/tools/myso3/myso3.cpp
        include/tools/myso3/myso3.hpp
)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gtsam)

add_subdirectory(test)