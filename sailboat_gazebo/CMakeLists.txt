cmake_minimum_required(VERSION 2.8.3)
project(sailboat_gazebo)

find_package(catkin REQUIRED COMPONENTS
  sailboat_description
  usv_gazebo_plugins
  xacro
  gazebo_ros
  gazebo_plugins
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS sailboat_description usv_gazebo_plugins xacro 
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
 include
  ${catkin_INCLUDE_DIRS}  ${GAZEBO_INCLUDE_DIRS} 
)

xacro_add_files(
  urdf/sailboat_gazebo.urdf.xacro
  INORDER INSTALL DESTINATION urdf
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

add_library(sailboat_gazebo src/LiftDragPluginNboat.cc)
add_library(sailboat_gazebo_sail src/LiftDragPluginNboatSail.cc)
target_link_libraries(sailboat_gazebo ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})