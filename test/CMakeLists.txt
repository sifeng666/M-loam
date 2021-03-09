#add_executable(test_factor test_factor.cpp)
#target_link_libraries(test_factor ${PROJECT_NAME})
#
#add_executable(test_align test_align.cpp point_cloud.h point_cloud.cpp visualization.h)
#target_link_libraries(test_align ${PROJECT_NAME})

#add_executable(test_plane_factor test_plane_factor.cpp)
#target_link_libraries(test_plane_factor ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
#
add_executable(test_map_gen test_map_gen.cpp ../include/tools/registration.hpp)
target_link_libraries(test_map_gen ${PCL_LIBRARIES} gtsam)

add_executable(test_global_align test_global_align.cpp ../include/tools/registration.hpp)
target_link_libraries(test_global_align ${PCL_LIBRARIES} fast_gicp)

add_executable(test_gtsam_opti test_gtsam_opti.cpp calibration_graph.cpp)
target_link_libraries(test_gtsam_opti gtsam)

add_executable(test_eigen test_eigen.cpp)
target_link_libraries(test_eigen gtsam)