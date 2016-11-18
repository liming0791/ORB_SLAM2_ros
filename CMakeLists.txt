cmake_minimum_required(VERSION 2.4.6)
project(orb_slam2)

set(ORB_SLAM2_DIR "/home/liming/ORB_SLAM2")
set(DBOW2_DIR ${ORB_SLAM2_DIR}/Thirdparty/DBoW2)
set(G2O_DIR ${ORB_SLAM2_DIR}/Thirdparty/g2o)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation cv_bridge image_transport tf sensor_msgs geometry_msgs genmsg)
find_package(cmake_modules REQUIRED)
find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen 3.0.0 REQUIRED)
find_package(Pangolin REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
#add_message_files(
#  FILES
#  IMU.msg
#)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetIMUPoseWorldService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_adafruit10dof
  CATKIN_DEPENDS message_runtime cv_bridge image_transport tf geometry_msgs
  DEPENDS Eigen
)

include_directories(
include
${PROJECT_SOURCE_DIR}
${ORB_SLAM2_DIR}
${ORB_SLAM2_DIR}/include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${catkin_LIBRARIES}
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${DBOW2_DIR}/lib/libDBoW2.so
${G2O_DIR}/lib/libg2o.so
${ORB_SLAM2_DIR}/lib/libORB_SLAM2.so
)

# Node for monocular camera
add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)


# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

# Node for Get IMU world Pose
add_executable(imu_pose_world
    src/imu_pose_world.cc)

target_link_libraries(imu_pose_world
${LIBS}
)
