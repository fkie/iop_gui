cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_mapviz_plugins)

set(DEPENDENCIES
  fkie_iop_msgs
  jaustoolset
  mapviz
  pluginlib 
  roscpp 
)

find_package(catkin REQUIRED COMPONENTS ${DEPENDENCIES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
set(QT_LIBRARIES
  ${Qt5Core_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)
include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)
add_definitions(
  ${Qt5Core_DEFINITIONS}
  ${Qt5Gui_DEFINITIONS}
  ${Qt5OpenGL_DEFINITIONS}
  ${Qt5Widgets_DEFINITIONS}
)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

find_package(OpenCV COMPONENTS core imgproc REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Qt5Core Qt5Gui Qt5OpenGL Qt5Widgets
  CATKIN_DEPENDS ${DEPENDENCIES}
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${PROJECT_BINARY_DIR}
)

set(UI_FILES
    ui/access_control_config.ui
    ui/access_control_dialog.ui
    ui/access_control_robot.ui
    ui/access_control_system_info.ui
    ui/access_control_warning_info.ui
    ui/digital_resource_config.ui
    ui/illuminator_config.ui
)

set(SRC_FILES
    src/access_control_client.cpp
    src/access_control_dialog.cpp
    src/access_control_plugin.cpp
    src/access_control_robot.cpp
    src/access_control_settings.cpp
    src/digital_resource_cam.cpp
    src/digital_resource_plugin.cpp
    src/illuminator_button.cpp
    src/illuminator_plugin.cpp
)

set(HEADER_FILES
    include/${PROJECT_NAME}/access_control_client.h
    include/${PROJECT_NAME}/access_control_dialog.h
    include/${PROJECT_NAME}/access_control_plugin.h
    include/${PROJECT_NAME}/access_control_robot.h
    include/${PROJECT_NAME}/access_control_settings.h
    include/${PROJECT_NAME}/digital_resource_cam.h
    include/${PROJECT_NAME}/digital_resource_plugin.h
    include/${PROJECT_NAME}/illuminator_button.h
    include/${PROJECT_NAME}/illuminator_plugin.h
)

qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_library(${PROJECT_NAME}
    ${MOC_FILES}
    ${SRC_FILES}
    ${UI_SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GLU_LIBRARY}
    ${GLUT_LIBRARY}
    ${OpenCV_LIBS}
    ${QT_LIBRARIES}
    ${QT_QTOPENGL_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS")

#add_dependencies(${PROJECT_NAME} marti_visualization_msgs_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
