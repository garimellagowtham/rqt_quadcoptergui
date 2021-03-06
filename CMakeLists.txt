cmake_minimum_required(VERSION 2.8.3)
project(rqt_quadcoptergui)

add_definitions( -DPRINT -std=c++0x)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  controllers
  dynamic_reconfigure
  parsernode
	dynamixelsdk
  roscpp
  rqt_gui
  rqt_gui_cpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
	gcop_comm
  gcop_ctrl
  message_generation
  tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
find_package(GCOP REQUIRED est systems)
find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED core highgui imgproc calib3d)
find_package(TinyXML REQUIRED)
find_package(Ceres 1.10 REQUIRED)

find_package(OpenMP)

if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
 catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   GuiStateMessage.msg
   GuiCommandMessage.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs   
   std_msgs   
   visualization_msgs
 )

## Generate dynamic reconfig options:
generate_dynamic_reconfigure_options(
  cfg/QuadcopterInterface.cfg
	  )
## Declare variables specific to rqt plugin:

set(rqt_quadcoptergui_SRCS
  src/rqt_quadcoptergui/quadcoptergui.cpp
	)

set(rqt_quadcoptergui_HDRS
  include/rqt_quadcoptergui/quadcoptergui.h
	)


set(rqt_quadcoptergui_UIS
  src/rqt_quadcoptergui/QuadCopterwidget.ui
	)

set(rqt_quadcoptergui_INCLUDE_DIRECTORIES
  include
	  ${CMAKE_CURRENT_BINARY_DIR}
		)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${rqt_quadcoptergui_INCLUDE_DIRECTORIES}
  LIBRARIES rqt_quadcoptergui
  CATKIN_DEPENDS controllers dynamic_reconfigure parsernode roscpp rqt_gui rqt_gui_cpp sensor_msgs std_msgs tf visualization_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${rqt_quadcoptergui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${TINYXML_INCLUDE_DIR} ${CERES_INCLUDE_DIRS}
)

#Plugin for QT
qt4_wrap_cpp(rqt_quadcoptergui_MOCS ${rqt_quadcoptergui_HDRS})
qt4_wrap_ui(rqt_quadcoptergui_UIS_H ${rqt_quadcoptergui_UIS})


## Declare a cpp library
# add_library(rqt_quadcoptergui
#   src/${PROJECT_NAME}/rqt_quadcoptergui.cpp
# )
add_library(${PROJECT_NAME} ${rqt_quadcoptergui_SRCS} ${rqt_quadcoptergui_MOCS} ${rqt_quadcoptergui_UIS_H})
# Onboard NodeHandler
add_library(onboardnodehandler src/onboardnodehandler.cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#message("GCOP LIBS: ${GCOP_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(onboardnodehandler ${GCOP_LIBRARIES} ${catkin_LIBRARIES} ${TINYXML_LIBRARIES} ${CERES_LIBRARY} ${CERES_LIBRARIES})
message("Catkin LIBS: ${catkin_LIBRARIES}")

## Declare a cpp executable
# add_executable(rqt_quadcoptergui_node src/rqt_quadcoptergui_node.cpp)
add_executable(onboard_node src/onboard_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(onboard_node rqt_quadcoptergui_gencfg)

## Specify libraries to link a library or executable target against
 target_link_libraries(onboard_node
   onboardnodehandler
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
# install(TARGETS rqt_quadcoptergui rqt_quadcoptergui_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rqt_quadcoptergui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
