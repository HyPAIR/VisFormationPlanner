cmake_minimum_required(VERSION 3.0.2)
project(vis_formation_planner)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

#ECBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

# Gurobi
set(GUROBI_HOME "/opt/gurobi1001/linux64")  # 替换为 Gurobi 安装路径
include_directories(${GUROBI_HOME}/include)
link_directories(${GUROBI_HOME}/lib)

add_definitions(-DHAVE_CSTDDEF -D_USE_MATH_DEFINES)

#Decomp
add_subdirectory(${PROJECT_SOURCE_DIR}/DecompROS/catkin_simple)
add_subdirectory(${PROJECT_SOURCE_DIR}/DecompROS/decomp_ros_msgs)
add_subdirectory(${PROJECT_SOURCE_DIR}/DecompROS/decomp_ros_utils)
include_directories(${PROJECT_SOURCE_DIR}/DecompROS/decomp_ros_utils/include)
include_directories(${PROJECT_SOURCE_DIR}/DecompROS/decomp_ros_msgs)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  base_local_planner
  costmap_2d
  gazebo_ros
  geometry_msgs
  nav_core
  nav_msgs
  std_msgs
  gazebo_msgs
  pluginlib
  tf2
  tf2_ros
  visualization_msgs
  interactive_markers
  octomap_ros
  octomap_msgs
  pcl_ros
  decomp_ros_utils
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(ompl REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vis_formation_planner
  CATKIN_DEPENDS gazebo_ros roscpp roslib message_runtime decomp_ros_utils
  DEPENDS OMPL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${OMPL_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  "/usr/include/python3.8/"
)
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Declare a C++ library
add_library(vis_formation_planner
  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/line_segment2d.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/visualization/color.cpp
  src/visualization/plot.cpp
  src/formation_planning/coarse_path_planner.cpp
  src/formation_planning/environment.cpp
  src/yaml.cpp
  src/formation_planning/robot_angle_optimizaer.cpp
  src/formation_planning/utils.cpp
)

add_library(dynamic_lines_mover SHARED src/formation_planning/dynamic_lines_mover.cpp)
target_link_libraries(dynamic_lines_mover ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(vis_formation_planner PUBLIC "/home/weijian/mosek/7/tools/platform/linux64x86/bin")
target_link_libraries(vis_formation_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ipopt adolc)
target_link_libraries(vis_formation_planner /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(vis_formation_planner gurobi_c++ gurobi100) 

add_library(camera_angle_controller SHARED src/formation_planning/camera_angle_controller.cpp)
target_link_libraries(camera_angle_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(write_obstalce_to_world src/formation_planning/writer_obs_to_world.cpp)
target_link_libraries(write_obstalce_to_world ${catkin_LIBRARIES} vis_formation_planner)
target_link_libraries(write_obstalce_to_world /usr/local/lib/libyaml-cpp.so.0.8)

add_executable(visable_region_test src/formation_planning/visable_region_test.cpp)
target_link_libraries(visable_region_test ${catkin_LIBRARIES} vis_formation_planner)
target_link_libraries(visable_region_test /usr/local/lib/libyaml-cpp.so.0.8)

# add_executable(test_decomp src/decomp_util/test_decomp.cpp)
# target_link_libraries(test_decomp ${Boost_LIBRARIES})
# target_link_libraries(test_decomp ${catkin_LIBRARIES} vis_formation_planner)
# target_link_libraries(test_decomp /usr/local/lib/libyaml-cpp.so.0.8)
# target_link_libraries(test_decomp
#   ${OCTOMAP_INCLUDE_DIRS}
#   ${SIPP_LINK_LIBS}
#   ${PYTHON_LIBRARIES}
#   m
#   ipopt
# )

add_executable(control_triangular_formation src/trajectory_tracking/fc_tracking_controller/control_triangular_formation.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(control_triangular_formation ${catkin_LIBRARIES} vis_formation_planner)
target_link_libraries(control_triangular_formation /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(control_triangular_formation PUBLIC /usr/include/eigen3)
target_link_libraries(control_triangular_formation /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(plot_result src/trajectory_tracking/plot_result.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(plot_result ${catkin_LIBRARIES})
target_link_libraries(plot_result /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(plot_result PUBLIC /usr/include/eigen3)
target_link_libraries(plot_result /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(plot_result3 src/trajectory_tracking/plot_result3.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(plot_result3 ${catkin_LIBRARIES})
target_link_libraries(plot_result3 /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(plot_result3 PUBLIC /usr/include/eigen3)
target_link_libraries(plot_result3 /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(control_rectangle_formation src/trajectory_tracking/fc_tracking_controller/control_rectangle_formation.cpp src/math/vec2d.cpp src/trajectory_tracking/plot.cpp)
target_link_libraries(control_rectangle_formation ${catkin_LIBRARIES})
target_link_libraries(control_rectangle_formation /usr/local/lib/libyaml-cpp.so.0.8)
target_include_directories(control_rectangle_formation PUBLIC /usr/include/eigen3)
target_link_libraries(control_rectangle_formation /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)

add_executable(animation_complete src/animation/complete_animation.cpp)
target_link_libraries(animation_complete ${catkin_LIBRARIES} vis_formation_planner)
target_link_libraries(animation_complete /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(animation_complete /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so)
target_link_libraries(animation_complete
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vis_formation_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
