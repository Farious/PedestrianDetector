# Copyright: TODO
# Authors: Fabio Reis, Matteo Taiana
# CopyPolicy: Licensed under the Simplified BSD License
cmake_minimum_required(VERSION 2.8.3)
project(peddetector)

## If compiling for system, no ROS compatibility, please uncomment the next 
## line. This will ensure that the make function will generate a binary to be 
## executed, without the ROS dependencies.
set(buildROS FALSE)

if(buildROS)
  message(Compiling\ for\ ROS.)
else(buildROS)
  message(Compiling\ for\ native.)
endif(buildROS)

if(buildROS)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
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
  INCLUDE_DIRS include
  LIBRARIES peddetector
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
  DEPENDS system_lib
)
endif(buildROS)

###########
## Build ##
###########

## Commands pretaining only the native version (non-ROS) of the pedestrian 
## detector:
if(NOT buildROS)
  SET(CMAKE_BUILD_TYPE RELEASE)
  SET(CMAKE_CXX_FLAGS_DEBUG   ""    )
  SET(CMAKE_CXX_FLAGS_RELEASE "")
#  INCLUDE_DIRECTORIES("/usr/local/include/")
endif(NOT buildROS)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

if(buildROS)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif(buildROS)

SUBDIRS(src)

## Declare a cpp library
# add_library(peddetector
#   src/${PROJECT_NAME}/peddetector.cpp
# )

## Declare a cpp executable
# add_executable(peddetector_node src/peddetector_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(peddetector_node peddetector_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(peddetector_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS peddetector peddetector_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_peddetector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
