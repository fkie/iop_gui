cmake_minimum_required(VERSION 2.8.3)
project(fkie_iop_rqt_velocity)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS python_qt_binding std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fkie_iop_rqt_velocity
  CATKIN_DEPENDS python_qt_binding std_msgs
#  DEPENDS system_lib
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
install(
    PROGRAMS 
        scripts/iop_rqt_velocity
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES 
        ./plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(
#   FILES
#        ./src/node_manager_fkie/MasterTab.ui
#        ./src/node_manager_fkie/GUI.qrc
#   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#)
