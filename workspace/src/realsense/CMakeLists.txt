cmake_minimum_required(VERSION 2.8.4)
project(realsense)
#include(~/dlib/build/dlib/config)
#include(~/dlib/dlib/all)
#include(~/dlib/cmake)

#include(3rdparty/dlib/cmake)

#set(dlib_DIR "~/dlib/build/dlib/config")
set(OpenCV_DIR "/home/shadylady/Programs/opencv-2.4.13/release")
#set(DLIB_PATH "" CACHE PATH "Path to DLIB")
#include(${DLIB_PATH}/dlib/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  roscpp
  message_generation
  std_msgs
  dynamic_reconfigure
  nodelet
  OpenCV
)
find_package(OpenCV REQUIRED)
#find_package(OpenGL REQUIRED)


#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   haar_face_detect.srv
#   cv_redetect.srv
#   )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense_face_node
#CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions pcl_ros roscpp sensor_msgs message_runtime  
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}

)


## Declare a C++ library
 add_library(hand_find
   src/handGesture.cpp
   src/myImage.cpp
   src/roi.cpp
   #src/fake_main.cpp
 )
  add_library(rsROSDevice
   src/rsROSDevice.cpp
 )
 
add_library(fake_main
   src/fake_main.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(rsROSDevice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(realsense_face_node_node src/realsense_face_node_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(realsense_face_node_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(rsROSDevice
   ${catkin_LIBRARIES}
   realsense
   ${OpenCV_LIBS}
 )
  target_link_libraries(fake_main
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   hand_find	
 )


add_executable(rs_main src/rs_main.cpp)
target_link_libraries(rs_main
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   rsROSDevice
   hand_find
   fake_main
   realsense
   dlib
)

add_executable(rs_main_OG src/rs_main_orig.cpp)
target_link_libraries(rs_main_OG
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   rsROSDevice
   hand_find
   fake_main
   realsense
   dlib
)

#add_executable(rs_main_OOP src/rs_main_OOP.cpp)
#target_link_libraries(rs_main_OOP
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
#   rsROSDevice
#   hand_find
#   realsense
#   dlib
#)
   
   
#add_executable(cv_redetect_node src/cv_redetect.cpp )
#add_dependencies(cv_redetect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cv_redetect_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# #  ${OPENGL_LIBRARIES}
#    realsense
#    dlib  
#   )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS rsROSDevice
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_realsense_face_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


# export LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
