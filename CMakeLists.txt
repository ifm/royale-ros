cmake_minimum_required(VERSION 2.8.3)
project(royale_ros)

set(ROYALE_ROOT "/opt/royale" CACHE STRING "Root directory to Royale SDK")

find_package(royale REQUIRED
  PATHS ${ROYALE_ROOT}/share
  )
find_package(Boost REQUIRED)

MESSAGE(STATUS "ROYALE_ROOT: " ${ROYALE_ROOT})
MESSAGE(STATUS "royale_LIB_DIR: " ${royale_LIB_DIR})
MESSAGE(STATUS "royale_LIBS: " ${royale_LIBS})

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  message_generation
  message_runtime
  cv_bridge
  image_transport
  pcl_ros
  sensor_msgs
  tf2_ros
  )

#######################################
## Declare ROS messages and services ##
#######################################
add_service_files(
  FILES
  Dump.srv
  Config.srv
  Start.srv
  Stop.srv
  )

add_message_files(
  FILES
  ExposureTimes.msg
  SetExposureTime.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_add_env_hooks(royale_env SHELLS sh)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES royale_ros
  CATKIN_DEPENDS roscpp nodelet
  )

#############
##  Build  ##
#############
# getting ugly warning coming out of ros/time.h, silence them for now
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion -std=c++14")

include_directories(
  include
  ${ROYALE_ROOT}/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
link_directories(
  ${royale_LIB_DIR}
  )

add_library(royale_ros src/camera_nodelet.cpp)
target_link_libraries(royale_ros
  ${catkin_LIBRARIES}
  ${royale_LIBS}
  )
add_dependencies(royale_ros ${PROJECT_NAME}_generate_messages_cpp)

add_executable(lscam src/lscam.cpp)
target_link_libraries(lscam
  ${royale_LIBS}
  )
add_dependencies(lscam ${PROJECT_NAME}_generate_messages_cpp)

add_executable(config src/config.cpp)
target_link_libraries(config
  ${catkin_LIBRARIES}
  )
add_dependencies(config ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dump src/dump.cpp)
target_link_libraries(dump
  ${catkin_LIBRARIES}
  )
add_dependencies(dump ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS
  royale_ros
  config
  dump
  lscam
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

######################
## Node-level tests ##
######################
