# Copyright: TODO
# Authors: Fabio Reis, Matteo Taiana
# CopyPolicy: Licensed under the Simplified BSD License

file(GLOB folder_source *.cpp)
file(GLOB folder_header ../include/*.hpp)

# Removing the ROS dependant files.
if(NOT buildROS)
  file(GLOB ros_source *_ROS.cpp)
  file(GLOB ros_header ../include/*_ROS.hpp)
  LIST(REMOVE_ITEM folder_source ${ros_source})
  LIST(REMOVE_ITEM folder_header ${ros_header})
else(NOT buildROS)
  file(GLOB native_source main.cpp)
  LIST(REMOVE_ITEM folder_source ${native_source})
endif(NOT buildROS)

# You may need to change this directory as to where your opencv libraries are.
# In my case they were in /usr/lib hence: /usr/lib/libopencv_*.so.
# This command it is only needed for the native version.
if(NOT buildROS)
  file(GLOB folder_opencv /usr/local/lib/libopencv_*.so)
endif(NOT buildROS)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(main ${folder_header} ${folder_source})

if(buildROS)
  target_link_libraries(main ${catkin_LIBRARIES})
else(buildROS)
  target_link_libraries(main ${folder_opencv})
endif(buildROS)

