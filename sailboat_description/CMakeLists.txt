cmake_minimum_required(VERSION 2.8.3)
project(sailboat_description)

find_package(catkin REQUIRED COMPONENTS xacro)
catkin_package(CATKIN_DEPENDS xacro)
xacro_add_files(
  urdf/sailboat_base.urdf.xacro
    INORDER INSTALL DESTINATION urdf 
)

install(DIRECTORY mesh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
