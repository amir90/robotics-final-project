# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project(Final_Project)

cmake_minimum_required(VERSION 2.8.11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


cmake_minimum_required(VERSION 2.8.11)
if(POLICY CMP0053)
  cmake_policy(SET CMP0053 OLD)
endif()
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 OLD)
endif()

find_package(CGAL COMPONENTS Qt5)

include(${CGAL_USE_FILE})

find_package(Qt5 QUIET COMPONENTS Xml Script OpenGL Svg)

if ( CGAL_FOUND AND CGAL_Qt5_FOUND AND Qt5_FOUND )

  add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_ui(arrangement_2_uis mainwindow.ui dialog.ui)

qt5_wrap_cpp( CGAL_Qt5_MOC_FILES mainwindow.h dialog.h)

  add_executable  ( min main.cpp CGAL_defines.h mainwindow.cpp dialog.cpp MyQueryHandler.cpp  IQueryHandler.h MyQueryHandler.h Path.cpp Path.h ${CGAL_Qt5_RESOURCE_FILES}${arrangement_2_uis} ${CGAL_Qt5_MOC_FILES})

  qt5_use_modules(min Xml Script OpenGL Svg Widgets)

  add_to_cached_list( CGAL_EXECUTABLE_TARGETS min )

  # Link with Qt libraries
  target_link_libraries( min ${QT_LIBRARIES} )
  # Link with CGAL
  target_link_libraries( min ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})

else()

  message(STATUS "NOTICE: This demo requires CGAL and Qt5, and will not be compiled.")

endif()
