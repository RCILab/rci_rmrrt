cmake_minimum_required(VERSION 3.10)
project(rmrrt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp roslib moveit_core moveit_ros_planning moveit_msgs geometric_shapes
                                        shape_msgs visualization_msgs tf2_ros)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rmrrt rmrrt_robot
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning moveit_msgs
  DEPENDS EIGEN3)

# The planner: metric, steering, node selection and RRT-Connect. Eigen only.
add_library(rmrrt
  src/metric/barrier.cpp
  src/metric/riemannian_metric.cpp
  src/geometry/distance.cpp
  src/geometry/capsule_margins.cpp
  src/steering/steering.cpp
  src/selection/selection.cpp
  src/planner/motion_tree.cpp
  src/planner/newton_projection.cpp
  src/planner/rrt_connect.cpp
  src/planner/method.cpp)
target_include_directories(rmrrt PUBLIC include)
target_link_libraries(rmrrt PUBLIC Eigen3::Eigen)
target_compile_options(rmrrt PRIVATE -Wall -Wextra)

# The robots: MoveIt scenes, the scenarios of config/scenarios.yaml and RViz.
add_library(rmrrt_robot
  src/robot/scenario.cpp
  src/robot/moveit_kinematics.cpp
  src/robot/robot_capsules.cpp
  src/robot/robot_scene.cpp
  src/robot/grasp_constraint.cpp
  src/robot/robot_problem.cpp
  src/robot/experiment.cpp
  src/robot/planner_config.cpp
  src/robot/robot_view.cpp)
target_include_directories(rmrrt_robot PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(rmrrt_robot PUBLIC rmrrt ${catkin_LIBRARIES} yaml-cpp)

foreach(app benchmark view)
  add_executable(${app} apps/${app}.cpp)
  target_link_libraries(${app} PRIVATE rmrrt_robot Boost::program_options)
endforeach()

add_executable(sphere_benchmark examples/sphere/sphere_benchmark.cpp)
target_link_libraries(sphere_benchmark PRIVATE rmrrt Boost::program_options)

add_executable(sphere_demo examples/sphere/sphere_demo.cpp)
target_include_directories(sphere_demo PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(sphere_demo PRIVATE rmrrt ${catkin_LIBRARIES} Boost::program_options)

if(CATKIN_ENABLE_TESTING)
  foreach(test test_metric test_steering test_geometry test_planner)
    catkin_add_gtest(${test} test/${test}.cpp)
    target_link_libraries(${test} rmrrt)
  endforeach()
endif()

install(TARGETS rmrrt rmrrt_robot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS benchmark view sphere_benchmark sphere_demo RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config launch robots scenes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
