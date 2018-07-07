# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project(Final_Project)

cmake_minimum_required(VERSION 2.8.11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(POLICY CMP0053)
  cmake_policy(SET CMP0053 OLD)
endif()
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 OLD)
endif()

# CGAL and its components
find_package(CGAL QUIET COMPONENTS)

if (NOT CGAL_FOUND)

    message(STATUS "This project requires the CGAL library, and will not be compiled.")
    return()

endif ()

# include helper file
include(${CGAL_USE_FILE})


# Boost and its components
find_package(Boost REQUIRED)

if (NOT Boost_FOUND)

    message(STATUS "This project requires the Boost library, and will not be compiled.")

    return()

endif ()


find_package(CGAL COMPONENTS Qt5)

include(${CGAL_USE_FILE})

find_package(Qt5 QUIET COMPONENTS Xml Script OpenGL Svg)

if ( CGAL_FOUND AND CGAL_Qt5_FOUND AND Qt5_FOUND )

  add_definitions(-DQT_NO_KEYWORDS)

  add_executable  (main main.cpp  ${CGAL_Qt5_RESOURCE_FILES} ${CGAL_Qt5_MOC_FILES})

  qt5_use_modules(main Xml Script OpenGL Svg)

  add_to_cached_list( CGAL_EXECUTABLE_TARGETS main )

  # Link with Qt libraries
  target_link_libraries( main ${QT_LIBRARIES} )
  # Link with CGAL
  target_link_libraries( main ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})

else()

  message(STATUS "NOTICE: This demo requires CGAL and Qt5, and will not be compiled.")

endif()


# include for local directory

# include for local package


# Creating entries for target: executable
# ############################

#set(SOURCE_FILES main.cpp CGAL_defines.h)
#add_executable(LM_RRT ${SOURCE_FILES})


#add_to_cached_list(CGAL_EXECUTABLE_TARGETS LM_RRT)

# Link the executable to CGAL and third-party libraries
#target_link_libraries(LM_RRT ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
