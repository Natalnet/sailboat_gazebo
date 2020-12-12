cmake_minimum_required(VERSION 2.8.3)
project(sailboat_teleop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  visualization_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
    DEPENDS
)

add_executable(sailboat_teleop src/sailboat_teleop.cpp)
target_link_libraries(sailboat_teleop ${catkin_LIBRARIES})
add_dependencies(sailboat_teleop std_msgs ${catkin_EXPORTED_TARGETS})

install(TARGETS sailboat_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)