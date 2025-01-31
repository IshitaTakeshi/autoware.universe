cmake_minimum_required(VERSION 3.16)
project(lidar_feature_localization)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-pg -g -O3 -Wall -Wextra -Wpedantic -Werror)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(fmt REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

set(INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rotationlib_INCLUDE_DIRS})

set(LIBRARY_SOURCE_FILES
  src/lib/algorithm.cpp
  src/lib/csv.cpp
  src/lib/eigen.cpp
  src/lib/pcl_utils.cpp
  src/lib/random.cpp
  src/lib/stats.cpp)

set(LOCALIZATION_SOURCE_FILES
  src/localization/alignment.cpp
  src/localization/debug.cpp
  src/localization/degenerate.cpp
  src/localization/edge.cpp
  src/localization/eigen.cpp
  src/localization/jacobian.cpp
  src/localization/kdtree.cpp
  src/localization/odometry_integration.cpp
  src/localization/optimizer.cpp
  src/localization/posevec.cpp
  src/localization/problem.cpp
  src/localization/robust.cpp
  src/localization/surface.cpp)

set(EXTRACTION_SOURCE_FILES
  src/extraction/convolution.cpp
  src/extraction/curvature.cpp
  src/extraction/index_range.cpp
  src/extraction/iterator.cpp
  src/extraction/label.cpp
  src/extraction/math.cpp
  src/extraction/ring.cpp)

set(ROS_DEPENDENT_SOURCE_FILES
  src/ros_dependent/map_receiver.cpp
  src/ros_dependent/ros_msg.cpp
  src/ros_dependent/ring.cpp)

add_library(library SHARED ${LIBRARY_SOURCE_FILES})
add_library(localization SHARED ${LOCALIZATION_SOURCE_FILES})
add_library(extraction SHARED ${EXTRACTION_SOURCE_FILES})
ament_auto_add_library(ros_dependent SHARED ${ROS_DEPENDENT_SOURCE_FILES})

target_include_directories(library PUBLIC ${INCLUDE_DIRS})
target_include_directories(localization PUBLIC ${INCLUDE_DIRS})
target_include_directories(extraction PUBLIC ${INCLUDE_DIRS})
target_include_directories(ros_dependent PUBLIC ${INCLUDE_DIRS})

target_link_libraries(library ${PCL_LIBRARIES} fmt)
target_link_libraries(localization ${PCL_LIBRARIES} fmt library)
target_link_libraries(extraction ${PCL_LIBRARIES} fmt library)
target_link_libraries(ros_dependent ${PCL_LIBRARIES})

ament_auto_add_executable("lidar_feature_localization" app/localization.cpp)
target_link_libraries("lidar_feature_localization" library localization extraction ros_dependent)

function(add_testcase filepath linked_libraries)
  string(REGEX REPLACE ".cpp" "" test_name ${filepath})
  string(REGEX REPLACE "/" "_" test_name ${test_name})

  ament_add_gmock(${test_name} ${filepath})

  target_link_libraries(${test_name} ${linked_libraries})

  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()


if(BUILD_TESTING)
  file(GLOB
    FILES_TO_CHECK
    ${CMAKE_CURRENT_SOURCE_DIR}/app/*
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${FILES_TO_CHECK})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${FILES_TO_CHECK})

  find_package(ament_cmake_gmock REQUIRED)

  set(LOCALIZATION_TEST_FILES
    test/localization/test_alignment.cpp
    test/localization/test_angular_integration.cpp
    test/localization/test_degenerate.cpp
    test/localization/test_edge.cpp
    test/localization/test_filter.cpp
    test/localization/test_jacobian.cpp
    test/localization/test_kdtree.cpp
    test/localization/test_math.cpp
    test/localization/test_odometry_integration.cpp
    test/localization/test_optimizer.cpp
    test/localization/test_pointcloud_to_matrix.cpp
    test/localization/test_recent_scans.cpp
    test/localization/test_robust.cpp
    test/localization/test_sliding_window.cpp
    test/localization/test_stamp_sorted_objects.cpp
    test/localization/test_surface.cpp
    test/localization/test_velocity_integration.cpp)

  foreach(filepath ${LOCALIZATION_TEST_FILES})
    add_testcase(${filepath} "library;localization")
  endforeach()

  set(EXTRACTION_TEST_FILES
    test/extraction/test_iterator.cpp
    test/extraction/test_algorithm.cpp
    test/extraction/test_convolution.cpp
    test/extraction/test_mapped_points.cpp
    test/extraction/test_curvature.cpp
    test/extraction/test_occlusion.cpp
    test/extraction/test_out_of_range.cpp
    test/extraction/test_range_message.cpp
    test/extraction/test_range.cpp
    test/extraction/test_neighbor.cpp
    test/extraction/test_label.cpp
    test/extraction/test_fill.cpp
    test/extraction/test_parallel_beam.cpp
    test/extraction/test_math.cpp
    test/extraction/test_ring.cpp
    test/extraction/test_index_range.cpp)

  foreach(filepath ${EXTRACTION_TEST_FILES})
    add_testcase(${filepath} "library;extraction")
  endforeach()

  set(LIB_TEST_FILES
    test/lib/test_algorithm.cpp
    test/lib/test_csv.cpp
    test/lib/test_degree_to_radian.cpp
    test/lib/test_eigen.cpp
    test/lib/test_numeric.cpp
    test/lib/test_pcl_utils.cpp
    test/lib/test_random.cpp
    test/lib/test_span.cpp
    test/lib/test_stats.cpp)

  foreach(filepath ${LIB_TEST_FILES})
    add_testcase(${filepath} "library")
  endforeach()

  set(ROS_DEPENDENT_TEST_FILES
    test/ros_dependent/test_map_reciever.cpp
    test/ros_dependent/test_ring.cpp
    test/ros_dependent/test_ros_msg.cpp)

  foreach(filepath ${ROS_DEPENDENT_TEST_FILES})
    add_testcase(${filepath} "ros_dependent")
  endforeach()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
