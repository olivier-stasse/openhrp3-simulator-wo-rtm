cmake_minimum_required(VERSION 2.6)

include(cmake/base.cmake)
include(cmake/pthread.cmake)
include(cmake/ros.cmake)

project(openhrp3_simulator_wo_rtm)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation std_srvs geometry_msgs sensor_msgs tf)

set(PROJECT_NAME openhrp3_simulator_wo_rtm)
set(PROJECT_DESCRIPTION "OpenHRP simulator")
set(PROJECT_URL "http://github.com/olivier-stasse/openhrp3-simulator-wo-rtm")

# Disable -Werror on Unix for now.
set(CXX_DISABLE_WERROR True)
set(CMAKE_VERBOSE_MAKEFILE True)

# Add std=c++11 flag
if (UNIX)
  SET(LFLAG -std=c++11)
  SET(CMAKE_CXX_FLAGS "${LFLAG} ${CMAKE_CXX_FLAGS}")
endif(UNIX)


setup_project()

catkin_package(CATKIN_DEPENDS message_runtime roscpp realtime_tools tf)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(OPENHRP_SIMULATOR_VERSION_MAJOR 3)
set(OPENHRP_SIMULATOR_VERSION_MINOR 1)
set(OPENHRP_SIMULATOR_VERSION_MICRO 3)

set(OPENHRP_LIBRARY_VERSION "${OPENHRP_SIMULATOR_VERSION_MAJOR}.${OPENHRP_SIMULATOR_VERSION_MINOR}.${OPENHRP_SIMULATOR_VERSION_MICRO}")

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
# Add local modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

include(FindPkgConfig)
include(CheckIncludeFiles)

# Check boost
find_package(Boost 1.35.0 REQUIRED COMPONENTS program_options filesystem signals system regex thread)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# check blas and lapack
find_package(Lapack REQUIRED)

include_directories(${LAPACK_INCLUDE_DIRS})
link_directories(${LAPACK_LIBRARY_DIRS})

# eigen
find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIR})

# png
if (UNIX)
  set(PNG_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/hrplib/hrpUtil/lpng1232" ) 
  include_directories(${PNG_INCLUDE_DIR})
  set(PNG_LIBRARY png)
endif()

# jpeg
if (UNIX)
  set(JPEG_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/hrplib/hrpUtil/Jpeg-6b" ) 
  include_directories(${JPEG_INCLUDE_DIR})
  set(JPEG_LIBRARY jpeg)
endif()

# zlib
if (UNIX)
  set(ZLIB_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/hrplib/hrpUtil/zlib123" )
  include_directories(${ZLIB_INCLUDE_DIR})
  set(ZLIB_LIBRARY zlib)
endif()

# omniORB
if(UNIX)
  if(NOT OMNIORB_DIR)
    pkg_check_modules(OMNIORB REQUIRED omniORB4)
    if(OMNIORB_FOUND)
      set(OMNIORB_DIR ${OMNIORB_PREFIX})
      message(STATUS "Found omniORB ${OMNIORB_VERSION} in ${OMNIORB_PREFIX}")
    endif()
  else()
    set(OMNIORB_INCLUDE_DIRS ${OMNIORB_DIR}/include)
    set(OMNIORB_LIBRARY_DIRS ${OMNIORB_DIR}/lib)
  endif()
endif()

# OnlineViewer
if(UNIX)
  add_required_dependency("gepetto-viewer")
  include_directories(${GEPETTO_VIEWER_INCLUDE_DIR})

  add_required_dependency("libxml-2.0")
endif()

include_directories(${PROJECT_BINARY_DIR}/hrplib)
include_directories(${PROJECT_SOURCE_DIR}/hrplib)

set(OPENHRP_SIMULATOR_IDL_DIR ${CMAKE_SOURCE_DIR}/idl)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
mark_as_advanced(LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH )

set(OPENHRP_IDL_DIR ${OpenHRP_SOURCE_DIR}/idl)
set(OPENHRP_DIR ${CMAKE_INSTALL_PREFIX})
set(OPENHRP_SHARE_DIR ${OPENHRP_DIR}/${RELATIVE_SHARE_INSTALL_PATH})

# Subdirectories
add_subdirectory(idl)
add_subdirectory(hrplib)
add_subdirectory(server)
add_subdirectory(sample)
add_subdirectory(scripts)

install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


setup_project_finalize()