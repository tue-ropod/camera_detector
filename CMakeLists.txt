cmake_minimum_required(VERSION 3.5)
project(camera_detector)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# add freenect2 kinect software library
find_library(LIBfreenect2 freenect2 /usr/local/lib)

# add openpose software library
find_library(LIBopenpose openpose /usr/local/lib)

# add google flags
find_library(LIBgflags gflags)

# add opencv
#set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/libs/opencv-install/lib/pkgconfig")
find_package(PkgConfig REQUIRED)
pkg_search_module(PKG_OPENCV REQUIRED opencv)  # this looks for opencv.pc file
include_directories(${PKG_OPENCV_INCLUDE_DIRS})

# find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  #geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY
   msg
   FILES
   detection.msg
   detections.msg
#   personAssociation.msg
#   person.msg
#   persons.msg
#   associationWouter.msg
#   associationsWouter.msg
#   circleProperties.msg
#   rectangleProperties.msg
#   objPosVel.msg
#   objsPosVel.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_detector
 CATKIN_DEPENDS roscpp std_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ~/ropodpc
  src
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/camera_detector.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/kinectDetector.cpp src/detection_processing.cpp)
add_executable(${PROJECT_NAME}_bagsim src/rosbagSimulation.cpp)

#add_executable(${PROJECT_NAME}_test src/hypothesize2.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}  ${LIBfreenect2} ${LIBopenpose} ${LIBgflags} ${PKG_OPENCV_LDFLAGS}
)
target_link_libraries(${PROJECT_NAME}_bagsim
  ${catkin_LIBRARIES}  ${LIBfreenect2} ${LIBopenpose} ${LIBgflags} ${PKG_OPENCV_LDFLAGS} 
)

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_camera_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
