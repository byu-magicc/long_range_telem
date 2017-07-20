cmake_minimum_required(VERSION 2.8.3)
project(long_range_telem)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosflight_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  rosflight
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp geometry_msgs rosflight_msgs sensor_msgs std_msgs
  DEPENDS Boost EIGEN3 YAML_CPP
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# long_range_telem_node
add_executable(long_range_telem
  src/long_range_telem_node.cpp
  src/long_range_telem.cpp
)
add_dependencies(long_range_telem
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rosflight_msgs_generate_msgs_cpp
)
target_link_libraries(long_range_telem
  mavrosflight
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and libraries for installation
install(TARGETS long_range_telem
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
