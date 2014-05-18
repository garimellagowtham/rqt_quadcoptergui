find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

set(rqt_quadcoptergui_SRCS
  quadcoptergui.cpp
	)

set(rqt_quadcoptergui_HDRS
  quadcoptergui.h
	)


set(rqt_quadcoptergui_UIS
  QuadCopterwidget.ui
	)

set(rqt_quadcoptergui_INCLUDE_DIRECTORIES
  ${PROJECT_SOURCE_DIR}/src
	  ${PROJECT_SOURCE_DIR}/build/src
		)

qt4_wrap_cpp(rqt_quadcoptergui_MOCS ${rqt_quadcoptergui_HDRS})
qt4_wrap_ui(rqt_quadcoptergui_UIS_H ${rqt_quadcoptergui_UIS})



rosbuild_include(pluginlib hide_plugin_symbols)
pluginlib_hide_plugin_symbols(rqt_quadcoptergui QuadcopterGui)

include_directories(${PROJECT_NAME} ${rqt_quadcoptergui_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rqt_quadcoptergui_SRCS} ${rqt_quadcoptergui_MOCS} ${rqt_quadcoptergui_UIS_H} )
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
