cmake_minimum_required(VERSION 3.0)

project(widget_Demo)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 QUIET COMPONENTS Core Gui Widgets )

include_directories(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/opencv4/include
	${PROJECT_BINARY_DIR})
	
set(srcs
	main.cpp
	widget_Demo.cpp
	trackingWorker.cpp
	monitoringWorker.cpp
)

set(hdrs
	widget_Demo.h
	trackingWorker.h
	monitoringWorker.h
)

set(uis
	widget_Demo.ui
)

qt5_wrap_ui(wrapUis ${uis})

add_executable(${PROJECT_NAME} ${srcs} ${hdrs} ${wrapUis} picture.qrc)
add_compile_definitions(ARMDCOMBINEDAPI_EXPORTS)

find_library(RUITONG_T_LIBRARY ARMDstereotracking ${CMAKE_CURRENT_SOURCE_DIR}/lib/Release)
find_library(RUITONG_R_LIBRARY ARMDstereoreconstruction ${CMAKE_CURRENT_SOURCE_DIR}/lib/Release)

#opencv libss
find_library(OPENCV_WORLD_LIBRARY opencv_world452 ${CMAKE_CURRENT_SOURCE_DIR}/Opencv4/lib)

target_link_libraries(${PROJECT_NAME} 
	${RUITONG_T_LIBRARY}
	${RUITONG_R_LIBRARY}
	#opencv lib
	${OPENCV_WORLD_LIBRARY}

	Qt5::Core
	Qt5::Gui
	Qt5::Widgets
	)

set(DIRS_IN_SRC
	Tool
)
if(EXISTS ${CMAKE_SOURCE_DIR}/${DIRS_IN_SRC})
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory 
	${CMAKE_SOURCE_DIR}/${DIRS_IN_SRC}
	${CMAKE_CURRENT_BINARY_DIR}/${DIRS_IN_SRC})
endif()

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/MANIFESTUAC:\"level='requireAdministrator' uiAccess='false'\" /SUBSYSTEM:WINDOWS")