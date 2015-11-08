cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(OpenHRP-Simulator)

set(OPENHRP_SIMULATOR_VERSION_MAJOR 3)
set(OPENHRP_SIMULATOR_VERSION_MINOR 1)
set(OPENHRP_SIMULATOR_VERSION_MICRO 3)


set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
# Add local modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

include(FindPkgConfig)
include(CheckIncludeFiles)

# Check boost
find_package(Boost 1.35.0 REQUIRED COMPONENTS filesystem signals system regex)

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

include_directories(${PROJECT_BINARY_DIR}/hrplib)
include_directories(${PROJECT_SOURCE_DIR}/hrplib)

set(OPENHRP_SIMULATOR_IDL_DIR ${OpenHRP-Simulator_SOURCE_DIR}/idl)

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
