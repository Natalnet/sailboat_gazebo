cmake_minimum_required(VERSION 2.8.3)
project(sailboat_gazebo)
find_package(catkin REQUIRED COMPONENTS
  sailboat_description
  usv_gazebo_plugins
  xacro
)
catkin_package(
  CATKIN_DEPENDS sailboat_description usv_gazebo_plugins xacro
)

# Generate urdf files from xacro and install
xacro_add_files(
  urdf/sailboat_gazebo.urdf.xacro
  INORDER INSTALL DESTINATION urdf
)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)