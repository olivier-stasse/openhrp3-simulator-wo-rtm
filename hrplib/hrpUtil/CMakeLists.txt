
# Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
# All rights reserved. This program is made available under the terms of the
# Eclipse Public License v1.0 which accompanies this distribution, and is
# available at http://www.eclipse.org/legal/epl-v10.html
# Contributors:
# National Institute of Advanced Industrial Science and Technology (AIST)

# @author Shin'ichiro Nakaoka


ADD_SUBDIRECTORY(lpng1232)
ADD_SUBDIRECTORY(Jpeg-6b)
ADD_SUBDIRECTORY(zlib123)

set(sources
  EasyScanner.cpp
  Eigen3d.cpp
  Eigen4d.cpp
  MatrixSolvers.cpp
  UrlUtil.cpp
  VrmlNodes.cpp
  VrmlParser.cpp
  VrmlWriter.cpp
  TriangleMeshShaper.cpp
  Triangulator.cpp
  ImageConverter.cpp
  OnlineViewerUtil.cpp
)

set(headers
  config.h
  Referenced.h
  EasyScanner.h
  EigenTypes.h
  Tvmet3dTypes.h
  Tvmet3d.h
  Tvmet4d.h
  Tvmet2Eigen.h
  Eigen3d.h
  Eigen4d.h
  uBlasCommonTypes.h
  MatrixSolvers.h
  TimeMeasure.h
  UrlUtil.h
  VrmlNodes.h
  VrmlParser.h
  VrmlWriter.h
  TriangleMeshShaper.h
  ImageConverter.h
  OnlineViewerUtil.h
)

set(target hrpUtil-${OPENHRP_LIBRARY_VERSION})
  
add_library(${target} SHARED ${sources})

# interface version 0.0.0 - version 3.0.0 beta1
set(HRPUTIL_VERSION ${HRPSOVERSION}.0.0 )
set_target_properties(${target} PROPERTIES VERSION ${HRPUTIL_VERSION} SOVERSION ${HRPSOVERSION})

message("PNG_LIBRARY: ${PNG_LIBRARY}")
message("JPEG_LIBRARY: ${JPEG_LIBRARY}") 
message("ZLIB_LIBRARY: ${ZLIB_LIBRARY}")

target_link_libraries(${target}
  hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
  #boost_filesystem boost_signals
  ${Boost_FILESYSTEM_LIBRARY} ${Boost_SIGNALS_LIBRARY} ${Boost_SYSTEM_LIBRARY}
  #${Boost_LIBRARIES}
  ${PNG_LIBRARY} ${JPEG_LIBRARY} ${ZLIB_LIBRARY}
  ${OMNIORB_LIBRARIES} ${LAPACK_LIBRARIES}
  )

hrplib_install_macro(${target} ${HRPUTIL_VERSION})

install(FILES ${headers} DESTINATION ${RELATIVE_HEADERS_INSTALL_PATH}/hrpUtil)

if(BUILD_GOOGLE_TEST)
  add_executable(testEigen3d testEigen3d.cpp Eigen3d.cpp)
  target_link_libraries(testEigen3d gtest gtest_main pthread)
  set_target_properties(testEigen3d PROPERTIES COMPILE_FLAGS "-g -O0 -coverage" LINK_FLAGS "-g -O0 -coverage")
  add_test(testEigen3d ${EXECUTABLE_OUTPUT_PATH}/testEigen3d)
endif()
